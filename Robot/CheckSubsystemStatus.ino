/*
   check the subsystem is fitting with the requirements if set flag to adjust
*/

boolean CheckSubsystemStatus() {
  if (bitRead(toDo, toDoStraight)) {
    if (BNOMode != MODE_IMUPLUS)  {
      expectedBNOMode = MODE_IMUPLUS;
      bitWrite(expectedSubsystemStatus,  monitMagnetoStatusBit, 0);
      return false;
    }
    else if (!bitRead(monitSubsystemStatus, monitGyroStatusBit)) {
      bitWrite(expectedSubsystemStatus, monitGyroStatusBit, 1);
      bitWrite(expectedSubsystemStatus,  monitMagnetoStatusBit, 0);
      return false;
    }
    else if (bitRead(expectedSubsystemStatus, monitGyroforward) != bitRead(monitSubsystemStatus, monitGyroforward)) {
      return false;
    }
    return !PIDFirstLoop;
  }
  else if (bitRead(toDoDetail, toDoAlignUpdateNO)) {
    if (BNOMode != MODE_COMPASS)  {
      expectedBNOMode = MODE_COMPASS;
      bitWrite(expectedSubsystemStatus, monitGyroStatusBit, 0);
      return false;
    }
    else if (!bitRead(monitSubsystemStatus, monitMagnetoStatusBit)) {
      bitWrite(expectedSubsystemStatus,  monitMagnetoStatusBit, 1);
      bitWrite(expectedSubsystemStatus, monitGyroStatusBit, 0);
      return false;
    }
    else if ( bitRead(toDoDetail, toDoGyroRotation)) {
      if (BNOMode != MODE_IMUPLUS)  {
        expectedBNOMode = MODE_IMUPLUS;
        //      bitWrite(expectedSubsystemStatus, monitGyroStatusBit, 1);
        bitWrite(expectedSubsystemStatus,  monitMagnetoStatusBit, 0);
        return false;
      }
      else if (!bitRead(monitSubsystemStatus, monitGyroStatusBit)) {
        bitWrite(expectedSubsystemStatus, monitGyroStatusBit, 1);
        bitWrite(expectedSubsystemStatus,  monitMagnetoStatusBit, 0);
        return false;
      }
      return true;
    }
  }
  else if (bitRead(toDo, toDoAlign)) {
    if (BNOMode != MODE_COMPASS)  {
      expectedBNOMode = MODE_COMPASS;
      bitWrite(expectedSubsystemStatus, monitGyroStatusBit, 0);
      return false;
    }
    else if (!bitRead(monitSubsystemStatus, monitMagnetoStatusBit)) {
      bitWrite(expectedSubsystemStatus,  monitMagnetoStatusBit, 1);
      return false;
    }
    return true;
  }
  return true;
}
