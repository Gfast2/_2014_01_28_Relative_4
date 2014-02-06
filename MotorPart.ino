void MotorStart(int MotorID){
     if (M_Done[MotorID] == true){
       String MotorIDFS = String(MotorID); 
       Serial2.println("#"+MotorIDFS+"A");
       delay(10);
       while (Serial2.read() >= 0); //mySerial Buffer leeren
     M_Done[MotorID] = false;
     }
}

//------------------------------------------------------------------

void MotorStop(int MotorID){
     if (M_Done[MotorID] == false){
       String MotorIDFS = String(MotorID);      
       Serial2.println("#"+MotorIDFS+"S1");
       delay(10);
       while (Serial2.read() >= 0); //mySerial Buffer leeren
       M_Done[MotorID] = true;
     }
}

//------------------------------------------------------------------


void MotorHardStop(int MotorID){
     if (M_Done[MotorID] == false){
       String MotorIDFS = String(MotorID);      
       Serial2.println("#"+MotorIDFS+"S0");
       delay(10);
       while (Serial2.read() >= 0); //mySerial Buffer leeren
       M_Done[MotorID] = true;
     }
}

//------------------------------------------------------------------

void SetMotorRichtung(int MotorID, int RichtungM){
    Serial2.println("#"+String(MotorID)+"d"+String(RichtungM));
    delay(10);
    while (Serial2.read() >= 0); //mySerial Buffer leeren
    Richtung[MotorID] = RichtungM;
}

//------------------------------------------------------------------

void SetMotor_a(int MotorID, long a_Value){
     //Serial2.println("#"+String(MotorID)+"b"+String(a_Value));
     //delay(10);
     
}


//------------------------------------------------------------------

void SetMotor_vMax(int MotorID, long vMax_Value){
     //Serial2.println("#"+String(MotorID)+"o"+String(vMax_Value));
     //delay(10);

     
}

//------------------------------------------------------------------

void notAus() {
  while(Serial.available()) {
    char n = Serial.read();
    if (n == 'a'){
      Serial2.println("#*S1");
      delay(20);
    } 
  }
}



//------------------------------------------------------------------

void SetMotorStartValues(){
  for(int i = 1; i<=4; i++){
    Serial2.println('#' + String(i) + "S0");
    delay(10); //10 works
    Serial2.println('#' + String(i) + "i30"); //set motors' performance //bei 497 ( 497 x 10 gram) 40% reicht schon nicht mehr aus. (still Phasenstrom set to 40%, too)
    delay(30); //10 works when single setup, when group setup, we need 30
    Serial2.println('#' + String(i) + "r30"); //set motorts' still performance
    delay(30); //10 works when single setup, when group setup, we need 30
    Serial2.println('#' + String(i) + "q0"); //Drehencoder direction exchange '0'-left go positive. '1' reversed.
    delay(10); //10 works
    Serial2.println('#' + String(i) + "g32"); //Schrittmodus einstellen (Vollschritt etc.)
    delay(250); //If smaller, can't get the next "1:ramp_mode = 0" echo
    Serial2.println('#' + String(i) + ":ramp_mode=0"); //Trapez-Rampe(0), Sinusrampe(1), Jerkfreerampe(2) Seite37
    delay(80); //10 works
    Serial2.println('#' + String(i) + "o7000"); //Maximalfrequenz, Maximalgeschwindigkeit einheit: Hertz/second Seite50
    delay(20); //Can't smaller, if so the next "#*u100" won't echo
    //Arduino Serial buffer not big enough to save all setting echo at beginning, so output some of them.
    //while(Serial2.available())
      //Serial.write(Serial2.read());
    Serial2.println('#' + String(i) + "u100"); //Minimalfrequenz Seite49
    delay(15); //Can't smaller, if so the next "#*p5" won't answer
    Serial2.println('#' + String(i) + "p5"); // '5' - Drehzahlmodus, '2' - "Absolute Positionierung" Seite47
    delay(10); //Can't smaller, if so the next "#*b41649" won't answer
    Serial2.println('#' + String(i) + "b41649"); //equal to 3Hz/ms, set acceleration Seite51
    //Serial2.println('#' + String(i) + "b19112"); //equal to 10Hz/ms
    //Serial2.println('#' + String(i) + "b32270"); //equal to 5Hz/ms
    delay(25); // Can't smaller, if so the next "#*B0" won't answer
    //Serial2.println('#' + String(i) + "B2364"); //50Hz/ms //set breaking acceleration
    //delay(25);
    Serial2.println('#' + String(i) + "B0");    //aktualisierte Bremsrampe setzen
    delay(10); // Can'tsmaller, if so the next "    
    Serial2.println('#' + String(i) + "D0"); //Dreh��berwachung Reset
    delay(10);
    Serial2.println('#' + String(i) + "d1");//'0' - links, '1' - rechts
    delay(10);
    //while(Serial2.available())
      //Serial.write(Serial2.read());
    //Serial.println();
    //Serial2.print("#*A\r");
    delay(5);
  }
  /*
  Serial2.println("#*S0");
  delay(10);
  Serial2.println("#*i30"); //set motors' performance
  delay(10);
  Serial2.println("#*S"); //motor stop
  delay(10);
  Serial2.println("#*q0"); //Drehencoder direction exchange
  delay(10);
  Serial2.println("#*g32"); //Schrittmodus einstellen (Vollschritt etc.)
  delay(10);
  Serial2.println("#*:ramp_mode=0"); //Trapez-Rampe(0), Sinusrampe(1), Jerkfreerampe(2) Seite37
  delay(10);
  Serial2.println("#*p5"); // '5' - Drehzahlmodus, '2' - "Absolute Positionierung" Seite47
  delay(10);
  Serial2.println("#*u100"); //Minimalfrequenz Seite49
  delay(10);
  Serial2.println("#*o7000"); //Maximalfrequenz Seite50
  delay(10);
  Serial2.println("#*b41649"); //equal to 3Hz/ms, set acceleration Seite51
  delay(10);
  Serial2.println("#1B0");    //aktualisierte Bremsrampe setzen
  delay(10);
  Serial2.println("#*d1");//'0' - links, '1' - rechts
  //delay(300000);
  while(Serial2.available()){
    Serial.print(Serial2.read());
  } 
  while (Serial2.read() >= 0); //mySerial Buffer leeren
  */
}
