/* SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2021 Avery Black
 * Ported to macOS from linux kernel, original source at
 * https://github.com/torvalds/linux/blob/master/drivers/input/rmi4/rmi_2d_sensor.c
 *
 * Copyright (c) 2011-2016 Synaptics Incorporated
 * Copyright (c) 2011 Unixphere
 */

#include "RMITrackpadFunction.hpp"
#include "RMILogging.h"
#include "RMIMessages.h"
#include "VoodooInputMultitouch/VoodooInputTransducer.h"
#include <IOKit/IOLib.h>

OSDefineMetaClassAndStructors(RMITrackpadFunction, RMIFunction)
#define super RMIFunction

#define RMI_2D_MAX_Z 140
#define RMI_2D_MIN_ZONE_VEL 10
#define RMI_2D_MIN_ZONE_Y_VEL 6
#define RMI_MT2_MAX_PRESSURE 255
#define cfgToPercent(val) ((double)val / 100.0)

// Coordinate smoothing: EMA factor (0.0 = full smoothing, 1.0 = no smoothing)
#define RMI_SMOOTHING_ALPHA 0.7
// Skip smoothing if distance exceeds this (large intentional movement)
#define RMI_SMOOTHING_MAX_JUMP 100

    static void fillZone(RMI2DSensorZone *zone, int min_x, int min_y, int max_x,
                         int max_y) {
  zone->x_min = min_x;
  zone->y_min = min_y;
  zone->x_max = max_x;
  zone->y_max = max_y;
}

void RMITrackpadFunction::setData(const Rmi2DSensorData &data) {
  this->data = data;
}

const Rmi2DSensorData &RMITrackpadFunction::getData() const { return data; }

bool RMITrackpadFunction::start(IOService *provider) {
  memset(freeFingerTypes, true, sizeof(freeFingerTypes));
  freeFingerTypes[kMT2FingerTypeUndefined] = false;

  for (size_t i = 0; i < MAX_FINGERS; i++) {
    fingerState[i] = RMI_FINGER_LIFTED;
    trackedFingers[i].active = false;
    trackedFingers[i].hasSmoothedCoords = false;
  }

  const RmiConfiguration &conf = getConfiguration();
  const int palmRejectWidth = data.maxX * cfgToPercent(conf.palmRejectionWidth);
  const int palmRejectHeight =
      data.maxY * cfgToPercent(conf.palmRejectionHeight);
  const int trackpointRejectHeight =
      data.maxY * cfgToPercent(conf.palmRejectionHeightTrackpoint);

  /*
   * Calculate reject zones.
   * These zones invalidate any fingers within them when typing
   * or using the trackpoint. 0, 0 is top left
   */

  // Top left
  fillZone(&rejectZones[0], 0, 0, palmRejectWidth, palmRejectHeight);

  // Top right
  fillZone(&rejectZones[1], data.maxX - palmRejectWidth, 0, data.maxX,
           palmRejectHeight);

  // Top band for trackpoint and buttons
  fillZone(&rejectZones[2], 0, 0, data.maxX, trackpointRejectHeight);

  // VoodooPS2 keyboard notifs
  setProperty("RM,deliverNotifications", kOSBooleanTrue);

  for (int i = 0; i < VOODOO_INPUT_MAX_TRANSDUCERS; i++) {
    auto &transducer = inputEvent.transducers[i];
    transducer.type = FINGER;
    transducer.supportsPressure = true;
    transducer.isValid = 1;
  }

  return super::start(provider);
}

IOReturn RMITrackpadFunction::message(UInt32 type, IOService *provider,
                                      void *argument) {
  switch (type) {
  case kHandleRMIClickpadSet:
    clickpadState = !!(argument);
    break;
  case kHandleRMITrackpoint:
    uint64_t timestamp;
    clock_get_uptime(&timestamp);
    absolutetime_to_nanoseconds(timestamp, &lastTrackpointTS);
    invalidateFingers();
    break;
  // VoodooPS2 Messages
  case kKeyboardKeyPressTime:
    lastKeyboardTS = *((uint64_t *)argument);
    invalidateFingers();
    break;
  case kKeyboardGetTouchStatus: {
    bool *result = (bool *)argument;
    *result = trackpadEnable;
    break;
  }
  case kKeyboardSetTouchStatus:
    trackpadEnable = *((bool *)argument);
    break;
  }

  return kIOReturnSuccess;
}

bool RMITrackpadFunction::shouldDiscardReport(AbsoluteTime timestamp) {
  return !trackpadEnable;
}

// Returns zone that finger is in (or 0 if not in a zone)
size_t RMITrackpadFunction::checkInZone(VoodooInputTransducer &obj) {
  TouchCoordinates &fingerCoords = obj.currentCoordinates;
  for (size_t i = 0; i < 3; i++) {
    RMI2DSensorZone &zone = rejectZones[i];
    if (fingerCoords.x >= zone.x_min && fingerCoords.x <= zone.x_max &&
        fingerCoords.y >= zone.y_min && fingerCoords.y <= zone.y_max) {
      return i + 1;
    }
  }

  return 0;
}

/**
 * RMI2DSensor::handleReport
 * Takes a report from F11/F12 and converts it for VoodooInput
 * This also does some input rejection.
 * There are three zones on the left, right, and top of the trackpad. If a touch
 * starts in those zones, it is not counted until it exits all zones This also
 * does some sanity checks for very wide or very big touch inputs This checks
 * for force touch on Clickpads only, where the trackpad is able to be pressed
 * down.
 */
void RMITrackpadFunction::handleReport(RMI2DSensorReport *report) {
  int validFingerCount = 0;
  const RmiConfiguration &conf = getConfiguration();

  bool discardRegions = ((report->timestamp - lastKeyboardTS) <
                         (conf.disableWhileTypingTimeout * MILLI_TO_NANO)) ||
                        ((report->timestamp - lastTrackpointTS) <
                         (conf.disableWhileTrackpointTimeout * MILLI_TO_NANO));

  size_t maxIdx = report->fingers > MAX_FINGERS ? MAX_FINGERS : report->fingers;

  // Remap finger indices to maintain stable identity across frames
  remapFingerIndices(report);

  for (int i = 0; i < maxIdx; i++) {
    rmi_2d_sensor_abs_object obj = report->objs[i];

    bool isValidObj = obj.type == RMI_2D_OBJECT_FINGER ||
                      obj.type == RMI_2D_OBJECT_STYLUS ||
                      // Allow inaccurate objects as they are likely invalid,
                      // which we want to track still This can be a random
                      // finger or one which was lifted up slightly
                      obj.type == RMI_2D_OBJECT_INACCURATE;

    auto &transducer = inputEvent.transducers[i];
    transducer.isTransducerActive = isValidObj;
    transducer.secondaryId = i;

    // Finger lifted, make finger valid
    if (!isValidObj) {
      fingerState[i] = RMI_FINGER_LIFTED;
      trackedFingers[i].active = false;
      trackedFingers[i].hasSmoothedCoords = false;
      continue;
    }

    validFingerCount++;

    // Apply coordinate smoothing before assigning to transducer
    UInt16 smoothedX = obj.x;
    UInt16 smoothedY = obj.y;
    if (conf.coordinateSmoothingEnabled && fingerState[i] == RMI_FINGER_VALID) {
      applyCoordinateSmoothing(i, smoothedX, smoothedY);
    }

    transducer.isTransducerActive = true;
    transducer.previousCoordinates = transducer.currentCoordinates;
    transducer.currentCoordinates.width = obj.z / 2.0;
    transducer.timestamp = report->timestamp;

    transducer.currentCoordinates.x = smoothedX;
    transducer.currentCoordinates.y = data.maxY - smoothedY;

    // Update tracked finger position (use raw coordinates for tracking)
    trackedFingers[i].x = obj.x;
    trackedFingers[i].y = obj.y;
    trackedFingers[i].active = true;

    switch (fingerState[i]) {
    case RMI_FINGER_LIFTED:
      fingerState[i] = RMI_FINGER_STARTED_IN_ZONE;
      // Current position is starting position, make sure velocity is zero
      transducer.previousCoordinates = transducer.currentCoordinates;
      // Initialize smoothed coords for new finger
      trackedFingers[i].smoothedX = (double)obj.x;
      trackedFingers[i].smoothedY = (double)obj.y;
      trackedFingers[i].hasSmoothedCoords = true;

      /* fall through */
    case RMI_FINGER_STARTED_IN_ZONE: {
      size_t zone = checkInZone(transducer);
      bool becameValid = false;
      if (zone == 0) {
        fingerState[i] = RMI_FINGER_VALID;
        becameValid = true;
      }

      int velocityX = abs((int)transducer.currentCoordinates.x -
                          (int)transducer.previousCoordinates.x);
      int velocityY = abs((int)transducer.currentCoordinates.y -
                          (int)transducer.previousCoordinates.y);

      IOLogDebug("Velocity: %d %d Zone: %ld", velocityX, velocityY, zone);
      if (velocityX > RMI_2D_MIN_ZONE_VEL ||
          (zone == 3 && velocityY > RMI_2D_MIN_ZONE_Y_VEL)) {
        fingerState[i] = RMI_FINGER_VALID;
        becameValid = true;
      }

      // Fix zone transition jump: reset previous to current when becoming valid
      if (becameValid) {
        transducer.previousCoordinates = transducer.currentCoordinates;
      }
    }
      /* fall through */
    case RMI_FINGER_VALID:
      if (obj.z > RMI_2D_MAX_Z || obj.wx > conf.palmRejectionMaxObjWidth ||
          obj.wy > conf.palmRejectionMaxObjHeight ||
          obj.type == RMI_2D_OBJECT_INACCURATE) {

        fingerState[i] = RMI_FINGER_INVALID;
      }

      // Force touch emulation only works with clickpads (button underneath
      // trackpad) Lock finger in place and in force touch until lifted Checks
      // for VALID input before registering as force touch
      if (isForceTouch(obj.z) && fingerState[i] == RMI_FINGER_VALID) {
        fingerState[i] = RMI_FINGER_FORCE_TOUCH;
      }

      break;
    case RMI_FINGER_FORCE_TOUCH:
      if (!isForceTouch(obj.z)) {
        fingerState[i] = RMI_FINGER_VALID;
        transducer.currentCoordinates.pressure = 0;
        break;
      }

      transducer.currentCoordinates = transducer.previousCoordinates;
      transducer.currentCoordinates.pressure = RMI_MT2_MAX_PRESSURE;
      break;
    case RMI_FINGER_INVALID:
      break;
    }

    transducer.isTransducerActive =
        fingerState[i] != RMI_FINGER_STARTED_IN_ZONE &&
        fingerState[i] != RMI_FINGER_LIFTED;

    IOLogDebug("Finger num: %d (%s) (%d, %d) [Z: %u WX: %u WY: %u FingerType: "
               "%d Pressure: %d]",
               i, fingerState[i] != RMI_FINGER_INVALID ? "valid" : "invalid",
               obj.x, obj.y, obj.z, obj.wx, obj.wy, transducer.fingerType,
               transducer.currentCoordinates.pressure);
  }

  if (validFingerCount >= 4 && freeFingerTypes[kMT2FingerTypeThumb]) {
    setThumbFingerType(maxIdx, report);
  }

  bool isGesture = !discardRegions && validFingerCount > 2;

  // Second loop to get finger type and allow gestures
  for (size_t i = 0; i < maxIdx; i++) {
    auto &trans = inputEvent.transducers[i];

    if (isGesture && fingerState[i] == RMI_FINGER_STARTED_IN_ZONE) {
      trans.isTransducerActive = true;
    }

    if (trans.isTransducerActive) {
      if (trans.fingerType == kMT2FingerTypeUndefined) {
        trans.fingerType = getFingerType();
      }

      if (fingerState[i] == RMI_FINGER_INVALID) {
        freeFingerTypes[trans.fingerType] = true;
        trans.fingerType = kMT2FingerTypePalm;
      }
    } else {
      // Free finger
      freeFingerTypes[trans.fingerType] = true;
      trans.fingerType = kMT2FingerTypeUndefined;
    }
  }

  inputEvent.transducers[0].isPhysicalButtonDown = clickpadState;
  inputEvent.contact_count = maxIdx;
  inputEvent.timestamp = report->timestamp;

  sendVoodooInputPacket(kIOMessageVoodooInputMessage, &inputEvent);

  // Mark inactive slots in tracked fingers
  for (size_t i = maxIdx; i < MAX_FINGERS; i++) {
    trackedFingers[i].active = false;
    trackedFingers[i].hasSmoothedCoords = false;
  }

  for (int i = 0; i < VOODOO_INPUT_MAX_TRANSDUCERS; i++) {
    inputEvent.transducers[i].isTransducerActive = false;
  }
}

// Take the most obvious lowest fingers - otherwise take finger with greatest
// area
void RMITrackpadFunction::setThumbFingerType(size_t maxIdx,
                                             RMI2DSensorReport *report) {
  size_t lowestFingerIndex = -1;
  size_t greatestFingerIndex = -1;
  UInt32 minY = 0, secondLowest = 0;
  UInt32 maxDiff = 0;
  UInt32 maxArea = 0;

  const RmiConfiguration &conf = getConfiguration();

  for (size_t i = 0; i < maxIdx; i++) {
    auto &trans = inputEvent.transducers[i];
    rmi_2d_sensor_abs_object *obj = &report->objs[i];

    if (!trans.isTransducerActive)
      continue;

    if (trans.currentCoordinates.y > minY) {
      lowestFingerIndex = i;
      secondLowest = minY;
      minY = trans.currentCoordinates.y;
    }

    if (trans.currentCoordinates.y > secondLowest &&
        trans.currentCoordinates.y < minY) {
      secondLowest = trans.currentCoordinates.y;
    }

    if (obj->z > maxArea) {
      maxDiff = (obj->wy - obj->wx);
      maxArea = obj->z;
      greatestFingerIndex = i;
    }
  }

  if (minY - secondLowest < conf.minYDiffGesture || greatestFingerIndex == -1) {
    lowestFingerIndex = greatestFingerIndex;
  }

  if (lowestFingerIndex == -1) {
    IOLogError("LowestFingerIndex = -1 When there are 4+ fingers");
    return;
  }

  auto &trans = inputEvent.transducers[lowestFingerIndex];
  if (trans.fingerType != kMT2FingerTypeUndefined)
    freeFingerTypes[trans.fingerType] = true;

  trans.fingerType = kMT2FingerTypeThumb;
  freeFingerTypes[kMT2FingerTypeThumb] = false;
}

// Assign the first free finger (other than the thumb)
MT2FingerType RMITrackpadFunction::getFingerType() {
  for (MT2FingerType i = kMT2FingerTypeIndexFinger; i < kMT2FingerTypeCount;
       i = (MT2FingerType)(i + 1)) {
    if (freeFingerTypes[i]) {
      freeFingerTypes[i] = false;
      return i;
    }
  }

  return kMT2FingerTypeUndefined;
}

/**
 * RMI2DSensor::invalidateFingers
 * Invalidate fingers which are in zones currently
 * Used when keyboard or trackpoint send events
 */
void RMITrackpadFunction::invalidateFingers() {
  for (size_t i = 0; i < MAX_FINGERS; i++) {
    VoodooInputTransducer &finger = inputEvent.transducers[i];

    if (fingerState[i] == RMI_FINGER_LIFTED ||
        fingerState[i] == RMI_FINGER_INVALID)
      continue;

    if (checkInZone(finger) > 0)
      fingerState[i] = RMI_FINGER_INVALID;
  }
}

bool RMITrackpadFunction::isForceTouch(UInt8 pressure) {
  const RmiConfiguration &conf = getConfiguration();
  switch (conf.forceTouchType) {
  case RMI_FT_DISABLE:
    return false;
  case RMI_FT_CLICK_AND_SIZE:
    return clickpadState && pressure > conf.forceTouchMinPressure;
  case RMI_FT_SIZE:
    return pressure > conf.forceTouchMinPressure;
  }
}

/**
 * remapFingerIndices
 * Stabilizes finger identity across frames using nearest-neighbor matching.
 * RMI4 firmware may reassign finger array indices between frames,
 * especially when fingers are not parallel. This function reorders
 * the current report's objects to match the previous frame's slot assignment.
 */
void RMITrackpadFunction::remapFingerIndices(RMI2DSensorReport *report) {
  const RmiConfiguration &conf = getConfiguration();
  size_t maxIdx = report->fingers > MAX_FINGERS ? MAX_FINGERS : report->fingers;

  // Count active tracked fingers from previous frame
  int prevActiveCount = 0;
  for (size_t i = 0; i < MAX_FINGERS; i++) {
    if (trackedFingers[i].active)
      prevActiveCount++;
  }

  // Count active fingers in current report
  int currActiveCount = 0;
  for (size_t i = 0; i < maxIdx; i++) {
    if (report->objs[i].type == RMI_2D_OBJECT_FINGER ||
        report->objs[i].type == RMI_2D_OBJECT_STYLUS ||
        report->objs[i].type == RMI_2D_OBJECT_INACCURATE)
      currActiveCount++;
  }

  // Only remap when we have multiple fingers in both frames
  if (prevActiveCount < 2 || currActiveCount < 2)
    return;

  const UInt32 maxDistSq =
      (UInt32)conf.fingerTrackingMaxDistance * conf.fingerTrackingMaxDistance;

  // Build distance matrix and find best matches using greedy nearest-neighbor
  // matchMap[currIdx] = prevIdx (-1 if unmatched)
  int matchMap[MAX_FINGERS];
  bool prevUsed[MAX_FINGERS];
  memset(matchMap, -1, sizeof(matchMap));
  memset(prevUsed, false, sizeof(prevUsed));

  // Collect (distance, currIdx, prevIdx) pairs, sort by distance
  struct DistPair {
    UInt32 distSq;
    int currIdx;
    int prevIdx;
  };

  DistPair pairs[MAX_FINGERS * MAX_FINGERS];
  int pairCount = 0;

  for (size_t ci = 0; ci < maxIdx; ci++) {
    rmi_2d_sensor_abs_object &curr = report->objs[ci];
    if (curr.type != RMI_2D_OBJECT_FINGER &&
        curr.type != RMI_2D_OBJECT_STYLUS &&
        curr.type != RMI_2D_OBJECT_INACCURATE)
      continue;

    for (size_t pi = 0; pi < MAX_FINGERS; pi++) {
      if (!trackedFingers[pi].active)
        continue;

      int dx = (int)curr.x - (int)trackedFingers[pi].x;
      int dy = (int)curr.y - (int)trackedFingers[pi].y;
      UInt32 distSq = (UInt32)(dx * dx + dy * dy);

      if (distSq < maxDistSq && pairCount < MAX_FINGERS * MAX_FINGERS) {
        pairs[pairCount].distSq = distSq;
        pairs[pairCount].currIdx = (int)ci;
        pairs[pairCount].prevIdx = (int)pi;
        pairCount++;
      }
    }
  }

  // Simple insertion sort (small N, kernel-safe)
  for (int i = 1; i < pairCount; i++) {
    DistPair key = pairs[i];
    int j = i - 1;
    while (j >= 0 && pairs[j].distSq > key.distSq) {
      pairs[j + 1] = pairs[j];
      j--;
    }
    pairs[j + 1] = key;
  }

  // Greedy matching: assign closest pairs first
  bool currUsed[MAX_FINGERS];
  memset(currUsed, false, sizeof(currUsed));

  for (int p = 0; p < pairCount; p++) {
    int ci = pairs[p].currIdx;
    int pi = pairs[p].prevIdx;
    if (currUsed[ci] || prevUsed[pi])
      continue;
    matchMap[ci] = pi;
    currUsed[ci] = true;
    prevUsed[pi] = true;
  }

  // Check if any remapping is actually needed
  bool needsRemap = false;
  for (size_t i = 0; i < maxIdx; i++) {
    if (matchMap[i] != -1 && matchMap[i] != (int)i) {
      needsRemap = true;
      break;
    }
  }

  if (!needsRemap)
    return;

  IOLogDebug("Finger remap triggered");

  // Apply remapping: reorder report objects and carry over state
  rmi_2d_sensor_abs_object remappedObjs[MAX_FINGERS];
  finger_state remappedState[MAX_FINGERS];
  TrackedFinger remappedTracked[MAX_FINGERS];
  VoodooInputTransducer remappedTransducers[MAX_FINGERS];

  // Initialize with current state
  for (size_t i = 0; i < MAX_FINGERS; i++) {
    remappedObjs[i] =
        (i < maxIdx) ? report->objs[i] : rmi_2d_sensor_abs_object{};
    remappedState[i] = fingerState[i];
    remappedTracked[i] = trackedFingers[i];
    remappedTransducers[i] = inputEvent.transducers[i];
  }

  for (size_t ci = 0; ci < maxIdx; ci++) {
    int targetSlot = matchMap[ci];
    if (targetSlot == -1 || targetSlot == (int)ci)
      continue;

    // Move current finger data into the matched previous slot
    remappedObjs[targetSlot] = report->objs[ci];
    remappedState[targetSlot] =
        fingerState[targetSlot]; // Keep prev slot's state
    remappedTracked[targetSlot] =
        trackedFingers[targetSlot]; // Keep prev slot's tracking
    remappedTransducers[targetSlot] =
        inputEvent.transducers[targetSlot]; // Keep prev transducer

    // Clear original slot if not also a target
    bool isTarget = false;
    for (size_t j = 0; j < maxIdx; j++) {
      if (matchMap[j] == (int)ci) {
        isTarget = true;
        break;
      }
    }
    if (!isTarget) {
      remappedObjs[ci].type = RMI_2D_OBJECT_NONE;
      remappedState[ci] = RMI_FINGER_LIFTED;
      remappedTracked[ci].active = false;
      remappedTracked[ci].hasSmoothedCoords = false;
    }
  }

  // Write back remapped data
  for (size_t i = 0; i < MAX_FINGERS; i++) {
    if (i < maxIdx)
      report->objs[i] = remappedObjs[i];
    fingerState[i] = remappedState[i];
    trackedFingers[i] = remappedTracked[i];
    inputEvent.transducers[i] = remappedTransducers[i];
  }
}

/**
 * applyCoordinateSmoothing
 * Applies Exponential Moving Average (EMA) filtering to finger coordinates.
 * Alpha = 0.7 provides a good balance between responsiveness and smoothness.
 * Skips smoothing for large movements (intentional fast gestures).
 */
void RMITrackpadFunction::applyCoordinateSmoothing(int fingerIdx, UInt16 &x,
                                                   UInt16 &y) {
  TrackedFinger &tracked = trackedFingers[fingerIdx];

  if (!tracked.hasSmoothedCoords) {
    // First frame for this finger, initialize
    tracked.smoothedX = (double)x;
    tracked.smoothedY = (double)y;
    tracked.hasSmoothedCoords = true;
    return;
  }

  double dx = (double)x - tracked.smoothedX;
  double dy = (double)y - tracked.smoothedY;
  double distSq = dx * dx + dy * dy;
  double maxJumpSq = (double)RMI_SMOOTHING_MAX_JUMP * RMI_SMOOTHING_MAX_JUMP;

  if (distSq > maxJumpSq) {
    // Large movement — don't smooth, just snap to new position
    tracked.smoothedX = (double)x;
    tracked.smoothedY = (double)y;
  } else {
    // Apply EMA: smoothed = alpha * raw + (1 - alpha) * previous
    tracked.smoothedX = RMI_SMOOTHING_ALPHA * (double)x +
                        (1.0 - RMI_SMOOTHING_ALPHA) * tracked.smoothedX;
    tracked.smoothedY = RMI_SMOOTHING_ALPHA * (double)y +
                        (1.0 - RMI_SMOOTHING_ALPHA) * tracked.smoothedY;
  }

  x = (UInt16)(tracked.smoothedX + 0.5);
  y = (UInt16)(tracked.smoothedY + 0.5);
}
