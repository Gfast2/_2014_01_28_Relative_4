/* This is the first on github uploaded prototype. It should also be the final beta version
 *
 * Edite: 6 Feb. 2014
 * By Gfast
 *
 *
 */

#include <SoftwareSerial.h>
SoftwareSerial LEDSerial(20, 21);

//Pin Definition
int chipDriver = 2;                          //RS485 Treiber für Transmitter (HalbDuplex)
int EndSchalterPin[5]={0,40,42,52,50};
int MagnetPin[5]={0,51,53,41,43};

const long RelativMaxWeg = 12000;

const int G_Schwellwert = 2;                 //Gewichtsänderungen ab Wert * 10g werden registriert
const int Minimalgewicht = 10;

const int G_Max = 300;                       //Maximalgewicht (Wert*10g), für Skalierung Beschleunigung, V_Max und Fehlererkennung 
const int G_MaxChange = 100;                 //Bei spontaner Gewichtsänderung größer Wert stoppen die Motoren (Fehlererkennung) 
const long FaktorGewicht = 200;              //Umrechnung Gewichtsdifferenz auf Weg (GetStatus) (Bei 500g Gewichtsdifferenz maximale Abstände zwischen Schalen)
const float BremsFaktor = 4.; 
const float a_Min = 1.;                      //Minimalbeschleunigung in Hz/ms^2; war 2.
const float a_Max = 6.;                      //Maximalbeschleunigung in Hz/ms; 

int DoneFlag = 0;                            //Ratsel successed
int MotorOben = 1;                           //aktuell höchster Motor zur Übergabe der Fahrtcharakteristika an Motor4  
boolean GewichtChange4 = false;
int EndSchalter[5];                          //Endschalter status 
int KippStatus[5];                           //Zugmagnet status
boolean M_Done[5]={                          //motor noch laeuft - false, motor stoped - true
  true, true, true, true, true};    
int Richtung[5];

int Durchlauf[5];

float a_Ref;
float a_Aktuell[5];
long b_Value[5];
long a_Value[5];
const long v_Max = 40000; //war 25000       //Maximum Geschwindigkeit alle Schale

long Position[5]={0,0,0,0,0};               //Akutelle Position
long PositionAlt[5];
long PositionSoll[5];
const int Schwellwert = 150;                //Position unterschied zwischen alte und neue gelesen Position,Unit: steps?

//-----------------Für Berechnung aktuelle Geschwindigkeit und Bremsweg
/*
long Bremsweg[5];
float Speed[5];
long TimeAlt[5];
long TimeNeu[5];
*/
//-----------------

//Gewicht und Status-----
long Gewicht[5];
long GewichtAlt[5];
int G_Change = 0;
int StatusNeu;                               //eine von 15 unterschiedlich Status
int StatusAlt; 
boolean StatusChange = false;
//-----

long kippPosition = -45000;
long kippPosUnten = -80000;

long ErrorDist = -30000;

long PosOben = -35000;
long PosMitteOben = -45000;
long PosMitte = -55000;
long PosMitteUnten = -65000;
long PosUnten = -75000;

long AnschlagOben = -10000;
long AnschlagUnten = -93000;

int Stufenlaenge = 10000;

long WegW1, WegW2;                          //WegW1 for the Grosse Waage, WegW2 for the small Waage of Plate 2 and 3

long StartPosition[5]={                     //Position fuer jeder Schale am Anfang und nach Calibration
  0, PosMitteOben, PosMitteUnten, PosMitteUnten, PosMitte};


//------------------------------------------------------------------------------------
/*
 _______  _______ _________          _______ 
(  ____ \(  ____ \\__   __/|\     /|(  ____ )
| (    \/| (    \/   ) (   | )   ( || (    )|
| (_____ | (__       | |   | |   | || (____)|
(_____  )|  __)      | |   | |   | ||  _____)
      ) || (         | |   | |   | || (      
/\____) || (____/\   | |   | (___) || )      
\_______)(_______/   )_(   (_______)|/       
*/                                       
void setup()
{
  Serial.begin(115200);   //Rechner
  Serial2.begin(115200);  //Motoren
  Serial3.begin(115200);  //Gewichtstransmitter
  LEDSerial.begin(38400); //LED and Musik control (software)

  pinMode(chipDriver,OUTPUT);
  digitalWrite(chipDriver, LOW);

  for(int i=1; i<5; i++) {
    pinMode(MagnetPin[i], OUTPUT);  
    pinMode(EndSchalterPin[i], INPUT);
  }

  Serial.println("\nAwake!");  
  calibration();
}

//----------------------------------------------------------------------------------------------------------------------------------
/*
 _        _______  _______  _______ 
( \      (  ___  )(  ___  )(  ____ )
| (      | (   ) || (   ) || (    )|
| |      | |   | || |   | || (____)|
| |      | |   | || |   | ||  _____)
| |      | |   | || |   | || (      
| (____/\| (___) || (___) || )      
(_______/(_______)(_______)|/  
*/
void loop()
{
  //----------Check ob Neukalibirierung erforderlich (Berührung Endschalter oder lange Inaktiv)------------
  
  //Endschalter einlesen, wenn Kontakt dann Neukalibrierung  
  for (int i=1;i<=4;i++){
    EndSchalter[i] = digitalRead(EndSchalterPin[i]);
  }
    
  if ((EndSchalter[1] == 0)||(EndSchalter[2] == 0)||(EndSchalter[3] == 0)||(EndSchalter[4] == 0)) {  //wenn Endschalter Kontakt Neukalibrierung 
    calibration();
  }
  //-------------------------------------------------------------------------------------------------------------- 
  
  StatusChange = false;    //Statuswechsel-Flag auf NULL setzen  
  G_Change = 0;          //Gewichtswechsel-Flag auf NULL setzen
 
 //------------Gewicht einlesen und Gewichtsänderung feststellen-------------------------------------------------
 
  for (int j=1;j<=4;j++){
    
    weightRead(j);//Gewicht einlesen
    delay(5);//Pause zum nächsten Transmitter
    if (Gewicht[j] < 0){Gewicht[j] = 0;}
    
    if (abs(GewichtAlt[j]-Gewicht[j])>G_Schwellwert){
      G_Change = 1;
      
      if (abs(GewichtAlt[j]-Gewicht[j])>(G_MaxChange)){
        G_Change = 0;
        MotorHardStop(1);
        MotorHardStop(2);
        MotorHardStop(3);
        MotorHardStop(4);
      }      
      GewichtAlt[j] = Gewicht[j];  
    }      
  }
 
  //-----------Ende Gewicht einlesen---------------------------------------------------
 
  //-----------Begin Statusabfrage (neue SollPositionen für Motoren)------------------- 
 
  if ((G_Change == 1)&&(DoneFlag == false)){   //Wenn Gewichtsänderung und Ratsel nit geloest, dann Motor-Werte aktualisieren 
    //Waagezeit_inaktiv = millis();            //Falls StatusÄnderung Reset Waagezeit_inaktiv (für Restart nach bestimmter Inaktivzeit)
    
    StatusNeu = GetStatus(Gewicht[1], Gewicht[2], Gewicht[3], Gewicht[4]);  //Alle aktuellen Gewichtswerte an Funktion "Getstatus" übergeben
    
    if (StatusNeu == 15) StatusNeu = StatusAlt;                             //Wenn Minimalgewicht nicht erreicht Status belassen
     
    SetNewStatus(StatusNeu, Gewicht[4]);                                    //Aus Statusnummer jeweilige SollPositionen setzen
    StatusAlt = StatusNeu;
    StatusChange = true;
    
    for (int i=1; i<=4;i++){
      Durchlauf[i] = 0;        //Für Nachschwingdurchgänge beim Bewegen
    }
    
    /*if ((StatusNeu != StatusAlt) || (GewichtChange4 == true)){              //Wenn Statusänderung oder Gewicht4-Änderung(Error oder Errorreturn)
        
    SetNewStatus(StatusNeu, Gewicht[4]);                                  //Aus Statusnummer jeweilige SollPositionen setzen
    StatusAlt = StatusNeu;
    StatusChange = true;
    
    for (int i=1; i<=4;i++){
    Durchlauf[i] = 0;        //Für Nachschwingdurchgänge beim Bewegen
    }
    } */
    
    //UpdateMotorValues();
  }
    
  //-------------- Wenn Sonderfall Rätsel gelöst --------------
  
  else if (StatusNeu == 14){              //Problem geloest
    DoneFlag = 1;
    LEDSerial.print("blink;"); 
  }
  
  else if (StatusNeu == 13){
    //Waagezeit_inaktiv = millis();           //Falls StatusÄnderung Reset Waagezeit_inaktiv (für Restart nach bestimmter Inaktivzeit)  
  }
  
  
  //-------------- Ende Sonderfall --------------
   
  //------------------Ende StatusZuweisung--------------------------------------------------
  

  //------------------Begin Motorfahrt / Fahrtkontrolle-------------------------------------

  if ((StatusChange == true) || (M_Done[1] == false) || (M_Done[2] == false) || (M_Done[3] == false) || (M_Done[4] == false)){
    MotorFahrt();
  }
  
  if ((DoneFlag == 1) && (M_Done[1] == true) && (M_Done[2] == true) && (M_Done[3] == true) && (M_Done[4] == true)){
    delay(2000);
    LEDSerial.print("kerze;");       
    
    calibration();
    DoneFlag = 0;
    delay(1000);
  }
  
  if (G_Change == 1) UpdateMotorCharacter();    //Bei Gewichtsänderung MotorWerte (Beschleunigung, MaximalSpeed, Bremsbeschleunigung aktualisieren)
  
  
   
} //--------------------- End of loop ---------------------

