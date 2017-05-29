float64 latitude # degrees
float64 longitude # degrees
float64 altitude # meters

int32 vertAccuracy # mm
int32 horiAccuracy # mm

uint8 nbSats

bool NO_FIX # The GPS receiver does not have a fix at all
bool FIX_2D # The GPS receiver has a 2D fix only
bool FIX_3D # Full 3D fix reception on the GPS receiver
bool TIME_FIX_ONLY # The GPS receiver has a time only fix

bool validTOW # Valid Time of Week
bool validWKN # Valid week number
bool validUTC # Valid UTC (Leap Seconds already known)
