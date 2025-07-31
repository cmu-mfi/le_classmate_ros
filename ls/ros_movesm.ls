/PROG  ROS_MOVESM
/ATTR
OWNER		= MNEDITOR;
COMMENT		= "r3";
PROG_SIZE	= 776;
CREATE		= DATE 25-05-28  TIME 14:33:34;
MODIFIED	= DATE 25-07-17  TIME 14:14:36;
FILE_NAME	= ;
VERSION		= 0;
LINE_COUNT	= 37;
MEMORY_SIZE	= 1260;
PROTECT		= READ_WRITE;
TCD:  STACK_SIZE	= 0,
      TASK_PRIORITY	= 50,
      TIME_SLICE	= 0,
      BUSY_LAMP_OFF	= 0,
      ABORT_REQUEST	= 0,
      PAUSE_REQUEST	= 0;
DEFAULT_GROUP	= 1,*,*,*,*;
CONTROL_CODE	= 00000000 00000000;
/MN
   1:   ;
   2:  !init: not rdy, no ack ;
   3:  F[1:OFF:ROS_I1]=(OFF) ;
   4:  F[2:OFF:ROS_I2]=(OFF) ;
   5:   ;
   6:  LBL[10] ;
   7:   ;
   8:  IF (DO[20:OFF:ArcTool Weld Start]=ON) THEN ;
   9:  Weld Start[1,1] ;
  10:  DO[20:OFF:ArcTool Weld Start]=OFF ;
  11:  JMP LBL[55] ;
  12:  ENDIF ;
  13:   ;
  14:  IF (DO[21:ON :ArcTool Weld End]=ON) THEN ;
  15:  Weld End[1,1] ;
  16:  DO[21:ON :ArcTool Weld End]=OFF ;
  17:  ENDIF ;
  18:   ;
  19:  !we're ready for a new point ;
  20:  LBL[55] ;
  21:  F[1:OFF:ROS_I1]=(ON) ;
  22:   ;
  23:  !wait for relay ;
  24:  WAIT (F[2:OFF:ROS_I2])    ;
  25:   ;
  26:  !cache in temp preg ;
  27:  PR[2]=PR[1]    ;
  28:   ;
  29:  !first rdy low, then ack copy ;
  30:  F[1:OFF:ROS_I1]=(OFF) ;
  31:  F[2:OFF:ROS_I2]=(OFF) ;
  32:   ;
  33:  !move to point ;
  34:J PR[2] R[1:x counter]% CNT R[2:z counter]    ;
  35:   ;
  36:  !done, repeat ;
  37:  JMP LBL[10] ;
/POS
/END
