
// 接線設定
void readWeight1() {
  weight_RF = scale_RF.get_units(1);
}
void readWeight2() {
  weight_RR = scale_RR.get_units(1);
}
void readWeight3() {
  weight_LR = scale_LR.get_units(1);
}
void readWeight4() {
  weight_LF = scale_LF.get_units(1);
}
void readWeight5() {
  weight_LM = scale_LM.get_units(1);
}
void readWeight6() {
  weight_RM = scale_RM.get_units(1);
}

void weightValue() {
  total_weight = (weight_RF + weight_RR + weight_LR + weight_LF + weight_LM + weight_RM)/100;
  left_weight  = (weight_LR + weight_LF + weight_LM)/100;
  right_weight = (weight_RF + weight_RR + weight_RM)/100;
  front_weight = (weight_RF + weight_LF)/100;
  real_weight  = (weight_RR + weight_LR)/100;
#if SERIAL_PRINT
  a++;
  Serial.print("total_weight: ");Serial.print("\t");Serial.print(total_weight);Serial.print("\t");
  Serial.print("left_weight: ");Serial.print("\t");Serial.print(left_weight);Serial.print("\t");
  Serial.print("right_weight: ");Serial.print("\t");Serial.print(right_weight);Serial.print("\t");
  Serial.print("front_weight: ");Serial.print("\t");Serial.print(front_weight);Serial.print("\t");
  Serial.print("real_weight: ");Serial.print("\t");Serial.print(real_weight);Serial.print("\t");
  Serial.print("a= ");Serial.print("\t");Serial.print(a);Serial.print("\n");
#endif
}
