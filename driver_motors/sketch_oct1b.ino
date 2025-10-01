const int FwdPin_A = 10; 
const int BwdPin_A = 9;  
const int FwdPin_B = 6; 
const int BwdPin_B = 5;        
 
int MaxSpd = 100;               // Скорость, значение 0-255
 
void setup()
{
  pinMode(FwdPin_A, OUTPUT);    // Устанавливаем FwdPin_A как выход
  pinMode(BwdPin_A, OUTPUT);    // Устанавливаем BwdPin_A как выход
  pinMode(FwdPin_B, OUTPUT);    // Устанавливаем FwdPin_B как выход
  pinMode(BwdPin_B, OUTPUT);    // Устанавливаем BwdPin_B как выход
}
 
void loop()
{
// Движение двигателем A вперед
    analogWrite(BwdPin_A,LOW);  
    analogWrite(FwdPin_A,MaxSpd); 
    delay(5000);
    analogWrite(FwdPin_A,LOW); 
// Движение двигателем A назад
    analogWrite(BwdPin_A,MaxSpd);
    analogWrite(FwdPin_A,LOW); 
    delay(5000);
    analogWrite(BwdPin_A,LOW);
// Движение двигателем B вперед
    analogWrite(BwdPin_B,LOW);  
    analogWrite(FwdPin_B,MaxSpd); 
    delay(5000);
    analogWrite(FwdPin_B,LOW); 
// Движение двигателем B назад
    analogWrite(BwdPin_B,MaxSpd);
    analogWrite(FwdPin_B,LOW); 
    delay(5000);
    analogWrite(BwdPin_B,LOW);
}
 