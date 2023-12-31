#include <avr/io.h>
#include "main.h"
#include "UART.h"
#include "maintenance.h"
#include "interface.h"
// Command: 
//    bit 7 = command flag
//    bit 6 =  ESC command
//    bit 5 =  maintenance mode
//    bit 4:2 = motor index (up to 8 motors)
//    bit 1:0 = setpoint bits 8 and 7 (up to 512 positions)


// setpoint
// bit 7 = 0 // not command
// bit 6:0 = setpoint bits 6:0

void getCommand()
{
  static int setPoitnt = 0;
  static int  msb = -1;
  
  int temp = getch();
  
 if(temp == -1) //nothing received
 {
    return ;
 }
 else if(FLAG(IS_MAINTENANCE)) //if we are in maintenance mode
 {
    doMaintenance(temp);
 }
 else if(IS_CMD_FOR_ME(temp)) // ESC setpoint command received
 {
     msb = temp & 0x3; //get two lower bits
 }
 else if (IS_MAINT(temp))
 {
    msb = -1;
    
    //is this a state request?
    if(temp & 0x01) // maint command with LSb set is a state report request
    {
      putch('N'); // we are at normal mode
      return;
    }
    SET_FLAG(IS_MAINTENANCE);
    putch('m'); // ack maintenance mode
    processSetPoint(0);
 }
 else if(IS_DATA(temp) && (msb != -1)) // if this byte is not a command and a command was just received before it
 {
   setPoitnt = (msb<<8) | temp; //update setpoint
   if(FLAG(GOV_MODE))
   {
     target_rpm = setPoitnt * RPM_PER_STEP;
   }
   else
   {
     processSetPoint(setPoitnt);
   }
   
   uart_timeout = UART_TOT;
   msb = -1; //reset as to wait for next command
 }  
 else // invalid sequence
 {
    msb = -1; //reset as to wait for next command
 }
 
 return ;
}

void processSetPoint(uint8_t setPoitnt)
{

     unsigned long pwmTemp = setPoitnt;   // "setPoitnt" has a range from 0 to 255
    // Limit power depending on motor RPM
    //look for startup conditions
    
    SET_FLAG(NEW_SETPOINT); //set flags to default values
    CLEAR_FLAG(FULL_POWER); 
    
    if(pwmTemp == 0) 
    {
       SET_FLAG(POWER_OFF);   // Setpoint=0 means Power off
       CLEAR_FLAG(NEW_SETPOINT); 
    }
    else if(FLAG(STARTUP)) //If we are in the Start-up sequence, limit  power to PWR_STARTUP
    {
        pwmTemp = (pwmTemp > STARTUP_POWER) ? STARTUP_POWER : pwmTemp;
    }
    else 
    {
      LIMIT_PWM(meas_rpm,pwmTemp); //limit PWM depending on motor RPM
    //  putch(RPM_CURVE(meas_rpm));
    }
    
    if(pwmTemp == 255) //If power is maximunm, set flags
    {
        SET_FLAG(FULL_POWER);
    }

   //  Compute PWM On and Off timer setings  
   
   //
   
    pwmTemp *= PWM_RANGE;  // reduce it to PWM_RANGE resolution
    pwmTemp /= INPUT_RANGE; 
    
    pwm_off_timer = (0xFF - PWM_RANGE) + ((uint8_t)pwmTemp);
    pwm_on_timer  = 0xFF - ((uint8_t)pwmTemp);
    

}
