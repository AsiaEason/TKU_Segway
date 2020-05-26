void MotorControl() {

  curr_l = constrain(analogRead(LEFT_CURR_SPEED), 61, 758); //0.3v to 3.7v
  curr_r = constrain(analogRead(RIGHT_CURR_SPEED), 61, 758);
  curr_l = -1*map(curr_l, 61, 758, -100, 100);
  curr_r = map(curr_r, 61, 758, -100, 100);

/*
  ref_vel = curr_l;
  err_vel = cmd_vel - ref_vel;
  div_err_vel = err_vel - err_vel_old;
  int_err_vel = int_err_vel + err_vel;
  if(int_err_vel>15)int_err_vel=15;
  if(int_err_vel<-15)int_err_vel=-15;
  float oPID_vel = 0.5*err_vel + 0*1*div_err_vel + 0*0.01*int_err_vel;
  cmd_theda = 0*oPID_vel;
  err_vel_old = err_vel;
*/

  if(fabs(Inclination)<0.01745*5) balance_mode=true;
  if(fabs(Inclination)>0.01745*8 && total_weight<=10) balance_mode=false;
  
  if(balance_mode)
  {
    digitalWrite(RIGHT_EN,HIGH);
    digitalWrite(LEFT_EN,HIGH);
    
    ref_theda = Inclination;
    err_theda = cmd_theda - ref_theda;
    div_err_theda = err_theda - err_theda_old;
    int_err_theda = int_err_theda + err_theda;
    if(fabs(int_err_theda)>15) int_err_theda=(int_err_theda/fabs(int_err_theda))*15;
    
    float oPID_theda = 50*err_theda + 100*div_err_theda + 1*int_err_theda;
    l = r = -oPID_theda*1.8*((total_weight+1000)/1000);//*1.8=40%
    err_theda_old = err_theda;
  
    if(left_weight>=10 && right_weight>=10) turn_mode=true;
    if(total_weight<=10) turn_mode=false;
    
    if(turn_mode)
    {
      turn_vel = (right_weight-left_weight)/total_weight*(fabs(oPID_theda)/MAX_SPEED*2);
      if(-oPID_theda<0) turn_vel = -turn_vel;
      l = l + 13*turn_vel;
      r = r - 13*turn_vel;
    }
  }
  else
  {
    digitalWrite(RIGHT_EN,LOW);
    digitalWrite(LEFT_EN,LOW);
    
    l = r = err_theda = div_err_theda = int_err_theda = 0;//reset oPID_theda;
  }
  /*Serial.print("oPID_theda=");Serial.print("\t");Serial.print(oPID_theda);
  Serial.print("oPID_theda=");Serial.print("\t");Serial.print(0);
  Serial.print("int_err_theda=");Serial.print("\t");Serial.print(0);
  Serial.print("oPID_theda=");Serial.print("\t");Serial.print(0);
  Serial.print("\n");*/


  // DIR
  int dir_l = (l < 0) ? 1 : 0;
  int dir_r = (r < 0) ? 0 : 1;
  digitalWrite(LEFT_DIR,  dir_l);
  digitalWrite(RIGHT_DIR, dir_r);
  // VEL
  if(fabs(l) > MAX_SPEED)
    l = (l/fabs(l))*MAX_SPEED;
  if(fabs(r) > MAX_SPEED)
    r = (r/fabs(r))*MAX_SPEED;
  float left_cmd = constrain(fabs(l), 0, MAX_SPEED);
  float right_cmd = constrain(fabs(r), 0, MAX_SPEED);
  left_cmd  = map(left_cmd, 0, 100, MIN_PWM, MAX_PWM);
  right_cmd = map(right_cmd, 0, 100, MIN_PWM, MAX_PWM);
  analogWrite(LEFT_PWM,  left_cmd);
  analogWrite(RIGHT_PWM, right_cmd);
#if SERIAL_PRINT
  c++;
  if(c%100 == 0){
    Serial.print("===============================================");Serial.print(" c= ");Serial.print("\t");Serial.print(c);Serial.print("\n");
  }
#endif
}
