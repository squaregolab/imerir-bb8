
const int Buzzer=2; 
int i=0,j=0,k=0; 
void setup()   { 
pinMode(Buzzer, OUTPUT); //met la broche en sortie 
} 


void loop(){ 
  /*
for (i=0; i<500; i++) {
  tone(Buzzer, i); 
  delay(1); 
} 
for (i=500; i<5000; i=i+5) {
  tone(Buzzer, i); 
  delay(1); 
} */

 tone(Buzzer, 300); 
  delay(500); 
  /*tone(Buzzer, 200); 
  delay(500); 
  tone(Buzzer, 300); 
  delay(500); 
  tone(Buzzer, 400); 
  delay(500); 
  tone(Buzzer, 500); 
  delay(500); 
  tone(Buzzer, 600); 
  delay(500); 
  tone(Buzzer, 700); 
  delay(500); */

  /*
for (i=5000; i>500; i=i-5) {
  tone(Buzzer, i); 
  delay(1); 
} // fin boucle for 
for (i=500; i=0; i--) {
  tone(Buzzer, i); 
  delay(1); 
} 
*/ 
} 
