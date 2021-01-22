#include <Pixi.h>
#include <SPI.h>
#include <math.h>

Pixi pixi;

const boolean toggled = true;

const byte VCF_VCA = 2; // Enveloppe for VCF and VCA
const byte ADSR = 4; // pot control
const byte SW_GTF = 3; // switch control
const byte MIDI_L[VCF_VCA]    = {  115,    116};
const byte HR_MIDI[VCF_VCA][ADSR] = {{ 44, 45, 46, 47}, { 48, 49, 50, 51}};
const byte MIDI_GTF [SW_GTF][VCF_VCA] = {{ 103,    108}, { 104,    109}, { 105,    110}};

const int channel = 1; // MIDI channel
const int D_PINS = 13; // number of Digital PINS for swith on/off
const int SW3_1 =  29;
const int SW3_2 =  31;
const int MIDI_SW3 =  102;
const int SW4_1 = 1;        // SW4_1/SW4_2 is logic control for VCF (12db/24db/hp/bp)
const int SW4_2 = 2;        //Switch on1/on2/on3/on4
const int MIDI_SW4 = 112;

const int DAC = 13;
const int PIXI = 2;
const int LFO_POT = 6;

const int SW2[D_PINS] =      {36, 34, 32, 30, 33, 35,  3, 17, 20, 7,   5,  23, 4};
const int MIDI_SW2[D_PINS] = {95, 94, 93, 92, 90, 89, 88, 87, 86, 85, 84, 83 , 82 };

const int MIDI_SW_LFO [PIXI][LFO_POT] = {
  { 97,  98,  96, 100, 99, 101 },//  CC control switch on/off LFO 
  {106, 111, 107,   0,  0,   0 }
};

const int SW_LFO_A[PIXI][LFO_POT] =     {
  { 39,  41,  16,  26,  24,  37},//  Switch LFO A (potentiometre panning, centre= fermé, gauche LFO A, droite LFO B)
  { 18,  19,  22,   0,  0,   0 }
};

const int SW_LFO_B[PIXI][LFO_POT] =     {
  { 25,  15,  19,  38,  14,  27},//  Switch LFO A (potentiometre panning, centre= fermé, gauche LFO A, droite LFO B)
  {  6,   8,  21,   0,  0,   0 } 
};

const int CC_MIDI[PIXI][DAC] = {
  {32, 34,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},//  DAC B
  {37, 76, 77, 74, 75, 43, 41, 40, 39, 42, 91, 36, 38} //  DAC A
};


const int CC_LFO[PIXI][LFO_POT] = {
  {67, 68, 66, 72, 71, 73},       ///DAC B
  {79, 81, 80,  0,  0,  0 }       ///DAC A
};

const int CC_WAVE [2] = {33, 35};               // CC des sélecteurs de formes d'onde

float         k_timer[VCF_VCA]         = {0, 0};  //

unsigned long TimerStartTop[VCF_VCA]   = {0, 0};  // instant en microsecondes du dernier evenement start ou stop
unsigned long TimeRefA[VCF_VCA]        = {0, 0};  // Duree en microsecondes de l'Attack
unsigned long TimeRefD[VCF_VCA]        = {0, 0};  // Duree en microsecondes du Decay
unsigned long TimeRefR[VCF_VCA]        = {0, 0};  // Duree en microsecondes du Release
unsigned long tempsreel[VCF_VCA];                 // Differentiel du Temps en microseconde entre l'evenement et le temps absolu
unsigned long  Offset[VCF_VCA]         = {0, 0};  // Amortissement des montees et descente du MCP

byte high_byte[17][33];
byte note1;
byte note2;
byte mode ;

byte          klog                     = 4;       // Courbure equivalente à celles des condensateurs
byte          Etat_module[VCF_VCA]     = {0, 0};  // 0=NOP      1=A       2=D     3=S     4=R
byte          Choix_forme[VCF_VCA]     = {0, 0};  // 0=LIN      1=LOG     2=EXP
byte          Choix_timer[VCF_VCA]     = {0, 0};  // 0=x1       1=x0.1    2=x10
byte          Choix_gate_in[VCF_VCA]   = {0, 0};  // 0=GATE_IN  1=TRIG_IN 2=LOOP
byte          DureeRef[3] = {5, 10, 60};
byte          Etat_SW [VCF_VCA][SW_GTF];

bool top = false;
bool bot = false;

int out_value;
int SINE[2];
int TRI[2];
int TRIZ[2];
int TRIY[2];
int SAW[2];
int SAWY[2];
int SAWZ[2];
int PULSE[2];
int PWM [2];

int LFOA[2][6];
int LFOB[2][6];
int LFO [2][6];

int LFO_state[PIXI][LFO_POT];
int LFO_level[PIXI][LFO_POT];

int POT[VCF_VCA][ADSR];
int AMT[VCF_VCA];
int LEVEL [VCF_VCA];
int OUT[VCF_VCA];
int  tension[VCF_VCA]                   = {0, 0};  // Tension courante en millivolts
int  tensionlin[VCF_VCA]                = {0, 0};  // Tension courante en millivolts
int  tensionlog[VCF_VCA]                = {0, 0};  // Tension courante en millivolts
int  memo_tension[VCF_VCA]              = {0, 0};  // Tension boucle-1 en millivolts
unsigned int  TensionRefS[VCF_VCA]      = {0, 0};  // Tension (mV) de reference du SUSTAIN
unsigned int  TensionStartTop[VCF_VCA]  = {0, 0};  // Tension (mV) du dernier evenement start ou stop

float bend;
float pitch_value;
float divider = 120;                    //diviseur 1V/octave



boolean       reset_ADSR  ;          // Redémarage des envellopes en mode DUO
boolean       Etat_gate_in    = 0;   // Copie de l'etat de l'entree gate  (1=on  0=off)
boolean       StartTop        = 0;   // Flag fugitif sur l'Etat_gate_in  (1=actif  0=deactive)
boolean state[D_PINS];

/////////////// Les DAC utilisé sont des MAX11300, il en a 2, PIXI A et PIXI B/////////////////////////

char CH_LFO[PIXI][LFO_POT]  = {
  { CHANNEL_2,  CHANNEL_3,  CHANNEL_8, CHANNEL_13, CHANNEL_15, CHANNEL_18 }, /////DAC B
  { CHANNEL_12, CHANNEL_13, CHANNEL_14,     0,         0,           0     }  /////DAC A
};

char CH_X[PIXI][DAC]  =  {
  {CHANNEL_0, CHANNEL_11,         0,         0,         0,          0,          0,          0,          0,         0,          0,          0,           0},/////DAC B
  {CHANNEL_19, CHANNEL_0, CHANNEL_1, CHANNEL_2,  CHANNEL_3,  CHANNEL_5,  CHANNEL_6,  CHANNEL_7, CHANNEL_8, CHANNEL_9, CHANNEL_16, CHANNEL_17,  CHANNEL_18} /////DAC A
};
                  /// VCO1 ------ VCO2 ///
char CH_Si[2]  = {CHANNEL_10, CHANNEL_19 };
char CH_Tr[2]  = {CHANNEL_9,  CHANNEL_17 };
char CH_Sa[2]  = {CHANNEL_7,  CHANNEL_16 };
char CH_Pu[2]  = {CHANNEL_5,  CHANNEL_14 };
char CH_Pw[2]  = {CHANNEL_6,  CHANNEL_4 };


void setup() {

  word Pixi_ID = 0;
  float Temp = 0;
  word test = 0;

  Pixi_ID = pixi.configA();

  if (Pixi_ID == 0x0424 ) {
    Serial.print("Found PIXI module ID: 0x");
    Serial.println(Pixi_ID, HEX);

    pixi.configChannelA ( CHANNEL_19, CH_MODE_DAC, 5, CH_0_TO_10P, 0 );
    pixi.configChannelA ( CHANNEL_0, CH_MODE_DAC, 5, CH_0_TO_10P, 0 );
    pixi.configChannelA ( CHANNEL_1, CH_MODE_DAC, 5, CH_0_TO_10P, 0 );
    pixi.configChannelA ( CHANNEL_2, CH_MODE_DAC, 5, CH_0_TO_10P, 0 );
    pixi.configChannelA ( CHANNEL_3, CH_MODE_DAC, 5, CH_0_TO_10P, 0 );

    pixi.configChannelA ( CHANNEL_5, CH_MODE_DAC, 5, CH_0_TO_10P, 0 );
    pixi.configChannelA ( CHANNEL_6, CH_MODE_DAC, 5, CH_0_TO_10P, 0 );
    pixi.configChannelA ( CHANNEL_7, CH_MODE_DAC, 5, CH_0_TO_10P, 0 );
    pixi.configChannelA ( CHANNEL_8, CH_MODE_DAC, 5, CH_0_TO_10P, 0 );
    pixi.configChannelA ( CHANNEL_9, CH_MODE_DAC, 5, CH_0_TO_10P, 0 );

    pixi.configChannelA ( CHANNEL_10, CH_MODE_DAC, 5, CH_0_TO_10P, 0 );
    pixi.configChannelA ( CHANNEL_11, CH_MODE_DAC, 5, CH_0_TO_10P, 0 );
    pixi.configChannelA ( CHANNEL_12, CH_MODE_DAC, 5, CH_0_TO_10P, 0 );
    pixi.configChannelA ( CHANNEL_13, CH_MODE_DAC, 5, CH_0_TO_10P, 0 );
    pixi.configChannelA ( CHANNEL_14, CH_MODE_DAC, 5, CH_0_TO_10P, 0 );

    pixi.configChannelA ( CHANNEL_16, CH_MODE_DAC, 5, CH_0_TO_10P, 0 );
    pixi.configChannelA ( CHANNEL_17, CH_MODE_DAC, 5, CH_0_TO_10P, 0 );
    pixi.configChannelA ( CHANNEL_18, CH_MODE_DAC, 5, CH_0_TO_10P, 0 );

  }

  Pixi_ID = pixi.configB();

  if (Pixi_ID == 0x0424 ) {
    Serial.print("Found PIXI module ID: 0x");
    Serial.println(Pixi_ID, HEX);

    pixi.configChannelB ( CHANNEL_0, CH_MODE_DAC, 5, CH_0_TO_10P, 0 );


    pixi.configChannelB ( CHANNEL_1, CH_MODE_DAC, 5, CH_0_TO_10P, 0 );
    pixi.configChannelB ( CHANNEL_2, CH_MODE_DAC, 5, CH_0_TO_10P, 0 );
    pixi.configChannelB ( CHANNEL_3, CH_MODE_DAC, 5, CH_0_TO_10P, 0 );
    pixi.configChannelB ( CHANNEL_4, CH_MODE_DAC, 5, CH_0_TO_10P, 0 );
    pixi.configChannelB ( CHANNEL_5, CH_MODE_DAC, 5, CH_0_TO_10P, 0 );
    pixi.configChannelB ( CHANNEL_6, CH_MODE_DAC, 5, CH_0_TO_10P, 0 );
    pixi.configChannelB ( CHANNEL_7, CH_MODE_DAC, 5, CH_0_TO_10P, 0 );
    pixi.configChannelB ( CHANNEL_8, CH_MODE_DAC, 5, CH_0_TO_10P, 0 );
    pixi.configChannelB ( CHANNEL_9, CH_MODE_DAC, 5, CH_0_TO_10P, 0 );
    pixi.configChannelB ( CHANNEL_10, CH_MODE_DAC, 5, CH_0_TO_10P, 0 );

    pixi.configChannelB ( CHANNEL_11, CH_MODE_DAC, 5, CH_0_TO_10P, 0 );

    pixi.configChannelB ( CHANNEL_12, CH_MODE_DAC, 5, CH_0_TO_10P, 0 );
    pixi.configChannelB ( CHANNEL_13, CH_MODE_DAC, 5, CH_0_TO_10P, 0 );
    pixi.configChannelB ( CHANNEL_14, CH_MODE_DAC, 5, CH_0_TO_10P, 0 );
    pixi.configChannelB ( CHANNEL_15, CH_MODE_DAC, 5, CH_0_TO_10P, 0 );
    pixi.configChannelB ( CHANNEL_16, CH_MODE_DAC, 5, CH_0_TO_10P, 0 );
    pixi.configChannelB ( CHANNEL_17, CH_MODE_DAC, 5, CH_0_TO_10P, 0 );
    pixi.configChannelB ( CHANNEL_18, CH_MODE_DAC, 5, CH_0_TO_10P, 0 );
    pixi.configChannelB ( CHANNEL_19, CH_MODE_DAC, 5, CH_0_TO_10P, 0 );

  }

  for (int i = 0; i < D_PINS; i++) {

    pinMode(SW2[i], OUTPUT);
    digitalWrite(SW2[i], LOW); // Toutes les pins out ont pour valeur LOW
  }


  pinMode(SW3_1, OUTPUT);
  digitalWrite(SW3_1, LOW); // Toutes les pins out ont pour valeur LOW
  pinMode(SW3_2, OUTPUT);
  digitalWrite(SW3_2, LOW);

  pinMode(SW4_1, OUTPUT);
  pinMode(SW4_2, OUTPUT);
  digitalWrite(SW4_1, LOW);
  digitalWrite(SW4_2, LOW);

  for (int i = 0; i < PIXI; i++) {
    for ( int k = 0 ; k < LFO_POT ; k++ ) {

      pinMode(SW_LFO_A[i][k], OUTPUT);
      pinMode(SW_LFO_B[i][k], OUTPUT);
      digitalWrite(SW_LFO_A[i][k], LOW);
      digitalWrite(SW_LFO_B[i][k], LOW); // Toutes les pins out ont pour valeur LOW
    }
  }
  usbMIDI.setHandleNoteOn(OnNoteOn);
  usbMIDI.setHandleNoteOff(OnNoteOff);
  usbMIDI.setHandlePitchChange(OnPitchChange);
  usbMIDI.setHandleControlChange (OnControlChange);
}

void loop() {

  usbMIDI.read();
  AUTOMATE();           // GESTION DE L'ENCHAINEMENT DES TACHES
  CALCUL();             // DETERMINATION DE LA TENSION DE SORTIE
  ECRIREDAC();          // ENVOIE DE LA TENSION DE SORTIE SUR LE DAC
  IHM();                // GESTION DES INVERSEURS ET POTENTIOMETRES
}

void OnControlChange (byte channel, byte control, byte value) {
  if ( control == 116) {
    mode = map(value, 0, 127, 0, 1);      //monophonic ou duophonic
  }
  if ( control == 117 ) {
    reset_ADSR = map(value, 0, 127, 0, 1);      //Reset ADSR DUO-MODE
  }
  for (int i = 0; i < PIXI ; i++) {
    for ( int k = 0 ; k < DAC ; k++ ) {
      if (control >= 64 && control <= 127) {
        out_value =  map(value, 0, 127, 0, 4095) ;  //Gestion des CC en 7bits
      }
      if (control == MIDI_L[i])
      { LEVEL[i] =  map(value, 0, 127, 0, 4092) ;
      }
      if (control >= 0 && control <= 31) {
        high_byte[channel][control] = value;
      }
      else if (control >= 32 && control <= 63) {
        out_value = (high_byte[channel][control - 32] << 7) + value;        //Gestion des CC en 14bits
        out_value = map(out_value, 0, 16383, 0, 4095);
      }
      if (control == CC_MIDI[0][k]) {
        pixi.writeAnalogB ( CH_X[0][k], out_value);
      }
      if (control == CC_MIDI[1][k]) {
        pixi.writeAnalogA ( CH_X[1][k], out_value);
      }
      for ( int j = 0 ; j < ADSR ; j++ ) {
        if (control == HR_MIDI[i][j]) {                                 //copie de l'etat des CC pour gestion des envellopes
          POT[i][j] =  map(out_value, 0, 4095, 0, 1023) ;
        }
      }
    }
    for ( int j = 0 ; j < SW_GTF ; j++ ) {
      if (control == MIDI_GTF[i][j])
      { Etat_SW[i][j] =  map(value, 0, 127, 0, 2) ;
        Choix_gate_in[i] = Etat_SW[0][i];                           //Switch des envellopes
        Choix_timer[i] = Etat_SW[1][i];
        Choix_forme[i] = Etat_SW[2][i];
      }
    }
//----------------------------------------------
//GESTION DES FORMES  (un CC permet de choisir l'onde entre sinus/triangle/saw/pulse/PWM)
//----------------------------------------------     
 
 //le changement se fait à l'aide de CV qui contrôle des VCA, de tel sorte que l'on évolue petit à petit d'une forme à l'autre//   
 
    for (int j = 0; j < 2 ; j++) {
      if (control == CC_WAVE[j]) {

        SINE[j] = TRIZ[j] = constrain(out_value, 0, 1023);
        SINE[j] = map( SINE[j], 0, 1023, 4095, 0);

        TRIZ[j] = map(TRIZ[j], 0, 1023, 0, 4095);         
        TRIY[j] = SAWZ[j] = constrain(out_value, 1023, 2047);
        TRIY[j] = map(TRIY[j], 1023, 2047, 0, 4095);
        TRI[j] = TRIZ[j] - TRIY[j];

        SAWZ[j] = map(SAWZ[j], 1023, 2047, 0, 4095);
        SAWY[j] = PULSE[j] = constrain(out_value, 2047, 3071);
        SAWY[j] = map( SAWY[j], 2047, 3071, 0, 4095);
        SAW[j] =  SAWZ[j] - SAWY[j];

        PULSE[j] = map(PULSE[j], 2047, 3071, 0, 4095);
        PWM[j] = constrain(out_value, 3071, 4095);
        PWM[j] = map(PWM[j], 3071, 4095, 0, 4095);

        pixi.writeAnalogB ( CH_Si[j], SINE[j]);
        pixi.writeAnalogB ( CH_Tr[j], TRI[j]);
        pixi.writeAnalogB ( CH_Sa[j], SAW[j]);
        pixi.writeAnalogB ( CH_Pu[j], PULSE[j]);
        pixi.writeAnalogB ( CH_Pw[j], PWM[j]);
      }
    }
//----------------------------------------------
//GESTION DU PANNING DES LFO PAR CV
//---------------------------------------------- 
  
    for ( int k = 0 ; k < LFO_POT ; k++ ) {
      if (control == CC_LFO[i][k]) {

        LFOA[i][k] = constrain(value, 0, 63);
        LFOA[i][k] = map(  LFOA[i][k], 0, 63, 4095, 0);
        LFOB[i][k] = constrain(value, 64, 127);
        LFOB[i][k] = map(  LFOB[i][k], 64, 127, 0, -4095);
        LFO [i][k] = LFOA[i][k] - LFOB[i][k];                                                           

        if ( CH_LFO[i][k] == CH_LFO[0][k]) {
          pixi.writeAnalogB ( CH_LFO[i][k], LFO[i][k]);
        }
        if ( CH_LFO[i][k] == CH_LFO[1][k]) {
          pixi.writeAnalogA ( CH_LFO[i][k], LFO[i][k]);
        }
        if (MIDI_SW_LFO[i][k] == control )
        {
          LFO_state [i][k] = map(value, 0, 127, 0, 1);
        }
        if (CC_LFO[i][k] == control )
        {
          LFO_level [i][k] = value ;
        }

        if (LFO_level [i][k] <= 62 && LFO_state [i][k] == 1  )
        { digitalWrite( SW_LFO_A [i][k], HIGH);
          digitalWrite( SW_LFO_B [i][k], LOW);
        }
        else {
          digitalWrite( SW_LFO_A [i][k], LOW);
        }

        if (LFO_level [i][k] >= 65 && LFO_state [i][k]  == 0 )
        { digitalWrite( SW_LFO_B [i][k], HIGH);
          digitalWrite( SW_LFO_A [i][k], LOW);
        }
        else {
          digitalWrite( SW_LFO_B [i][k], LOW);
        }
      }
    }
  }

  //// GESTION DES SWITCH ON/OFF////
  for (int i = 0; i < D_PINS ; i++)
  { if (MIDI_SW2[i] == control)
    { if (value >= 64)
      { digitalWrite(SW2[i], HIGH);
        state[i] = true;
      }        else {
        digitalWrite(SW2[i], LOW);
        state[i] = false;
      }
    }
  }
//----------------------------------------------
//GESTION DU SWITCH GLIDE
//---------------------------------------------- 
    

  if (MIDI_SW3 == control)
  { if (value >= 84)
    { digitalWrite(SW3_1, HIGH);
      digitalWrite(SW3_2, LOW);
    }
    else if (value < 84 && value > 42)
    { digitalWrite(SW3_1, LOW);
      digitalWrite(SW3_2, LOW);
    }
    else if (value < 42)
    { digitalWrite(SW3_1, LOW);
      digitalWrite(SW3_2, HIGH);
    }
  }
//----------------------------------------------
// GESTION DU SWITCH SELECTEUR DE PENTE DU VCF
//---------------------------------------------- 
  
  { if (MIDI_SW4 == control)
    { if (value >= 96)
      { digitalWrite(SW4_1, HIGH);
        digitalWrite(SW4_2, LOW);
      }
      else if (value < 96 && value > 64)
      { digitalWrite(SW4_1, LOW);
        digitalWrite(SW4_2, LOW);
      }
      else if (value < 64 && value > 32)
      { digitalWrite(SW4_1, LOW);
        digitalWrite(SW4_2, HIGH);
      }
      else if (value < 32)
      { digitalWrite(SW4_1, HIGH);
        digitalWrite(SW4_2, HIGH);
      }
    }
  }
}

//----------------------------------------------
// CLAVIER EN MODE MONOPHONIC
//----------------------------------------------

void OnNoteOn(byte channel, byte pitch, byte velocity) {
  if (mode == 0 && channel == 1 && velocity > 0) {
    pitch_value = constrain(pitch, 0, 120);
    pixi.writeAnalogB ( CHANNEL_1, ((pitch_value + bend) / divider) * 4095.0);        
    pixi.writeAnalogB ( CHANNEL_12, ((pitch_value + bend) / divider) * 4095.0);
    Etat_gate_in = 1;
    StartTop = 1;
    AMT[0] = map(velocity, 0, 127, 0, LEVEL[0]);
    AMT[1] = map(velocity, 0, 127, 0, LEVEL[1]);
  }
  else {
    Etat_gate_in = 0;
  }

//----------------------------------------------
// CLAVIER EN MODE DUOPHONIC
//----------------------------------------------
  
  if  (mode == 1 && top == false && channel == 1 && velocity > 0) {
    pitch_value = constrain(pitch, 0, 120);
    pixi.writeAnalogB ( CHANNEL_1, ((pitch_value + bend) / divider) * 4095.0);
    top = true;
    note1 = pitch ;
    Etat_gate_in = 1;
    StartTop = 1;
    AMT[0] = map(velocity, 0, 127, 0, LEVEL[0]);
    AMT[1] = map(velocity, 0, 127, 0, LEVEL[1]);
  }
  else if (mode == 1 && top == true && bot == false && channel == 1 && velocity > 0) {
    pitch_value = constrain(pitch, 0, 120);
    pixi.writeAnalogB ( CHANNEL_12, ((pitch_value + bend) / divider) * 4095.0);
    bot = true;
    note2 = pitch ;
    if (reset_ADSR = 1) {
      Etat_gate_in = 1;
      StartTop = 1;
      AMT[0] = map(velocity, 0, 127, 0, LEVEL[0]);
      AMT[1] = map(velocity, 0, 127, 0, LEVEL[1]);
    }
  }
}

void OnNoteOff(byte channel, byte pitch, byte velocity) {
  if (pitch == note1 && channel == 1) {
    top = false;
    Etat_gate_in = 0;
  }
  else if (pitch == note2 && channel == 1) {
    bot = false;
    Etat_gate_in = 0;
  }
}

void OnPitchChange (byte channel, int pitch_change) {
  if (channel == 1) {
    bend = map(pitch_change, 8191 , -8192, 0, 24);
  }
}
//----------------------------------------------
// ENVOI DE LA TENSION DE SORTIE AU DAC
//----------------------------------------------

void ECRIREDAC() {
  for (int i = 0; i < VCF_VCA ; i++) {
    if (memo_tension[i] != tension[i]) {
      memo_tension[i] = tension[i];

      OUT[i] = map(tension[i], 0, 4095, 0, AMT[i]);

      pixi.writeAnalogA ( CHANNEL_10, OUT[0]);
      pixi.writeAnalogA ( CHANNEL_11, OUT[1]);
    }
  }
}

//----------------------------------------------
// CALCUL DE LA TENSION DE SORTIE (0->4095mV)
//----------------------------------------------
void CALCUL() {
  for (int i = 0; i < VCF_VCA ; i++)
    //----------------------------------------------------------------------ATTACK--
    if (Etat_module[i] == 1) {
      tempsreel[i] = (micros() - TimerStartTop[i]);
      tension[i] = 4095 - TensionStartTop[i];
      tension[i] *= float(tempsreel[i] * k_timer[i] / (TimeRefA[i] * (4095.00 - TensionStartTop[i]) / 4095.00));
      tension[i] += TensionStartTop[i];
      if      (Choix_forme[i] == 1) {
        tension[i] += 512;
        float k = (4095 - TensionStartTop[i]) / 3.61;
        tension[i] = (log10(tension[i] - TensionStartTop[i]) * k * klog) + (4095 - (log10(4095 - TensionStartTop[i]) * k * klog));
      }
      if      (Choix_forme[i] == 2) {
        tensionlin[i] = tension[i];
        tension[i] = 4095 - TensionStartTop[i];
        float k = tension[i] / 3.61;
        tension[i] *= float((TimeRefA[i] - (tempsreel[i] * k_timer[i])) / (TimeRefA[i] * (4095.00 - TensionStartTop[i]) / 4095.00));
        tensionlog[i] = (log10(tension[i] - TensionStartTop[i]) * k * klog) + (4095 - (log10(4095 - TensionStartTop[i]) * k * klog));
        tension[i] = tensionlin[i] - tensionlog[i] + tension[i];

      }
      tension[i] = constrain(tension[i], TensionStartTop[i], 4095);
    }

  //----------------------------------------------------------------------DECAY--
    else if (Etat_module[i] == 2) {
      tempsreel[i] = (micros() - TimerStartTop[i]);
      tension[i] = TensionStartTop[i] - ((TensionStartTop[i] - TensionRefS[i]) * (k_timer[i] * tempsreel[i] / TimeRefD[i] * 2));
      if      (Choix_forme[i] == 1) { //LOG => EXP en descente
        float k = log(TensionStartTop[i] - TensionRefS[i]) * (1 - (k_timer[i] * tempsreel[i] / TimeRefD[i]));
        if (k > 16) {
          tension[i] = 4095;
        } else {
          tension[i] = exp(k) + TensionRefS[i] - 1;
        }
      }
      else if (Choix_forme[i] == 2) { // EXP => LOG en descente

        tension[i] = TensionRefS[i] + ((log10((TensionStartTop[i] - (TensionStartTop[i] * (k_timer[i] * tempsreel[i] / TimeRefD[i] * 2))) / (klog * 100))) * ((TensionStartTop[i] - TensionRefS[i]) / log10(TensionStartTop[i] / (klog * 100))));
      }
      tension[i] = constrain(tension[i], TensionRefS[i] - 1, 4095);
    }

  //----------------------------------------------------------------------SUSTAIN--
    else if (Etat_module[i] == 3) {
      tension[i] = TensionRefS[i];
    }

  //----------------------------------------------------------------------RELEASE--
    else if (Etat_module[i] == 4) {
      tempsreel[i] = (micros() - TimerStartTop[i]);
      tension[i] = TensionStartTop[i] - (k_timer[i] * tempsreel[i] * float(4095.00 / TimeRefR[i] * 2)); //  LIN
      if      (Choix_forme[i] == 1) {                                     //  LOG
        float k = log(TensionStartTop[i]) * (1 - (k_timer[i] * tempsreel[i] / TimeRefR[i]));
        if (k > 16) {
          tension[i] = 4095;
        } else {
          tension[i] = exp(k);
          tension[i] -= 50;
        }
      }
      else if (Choix_forme[i] == 2) {                                     //  EXP

        tension[i] = (log10((TensionStartTop[i] - (TensionStartTop[i] * (k_timer[i] * tempsreel[i] / TimeRefR[i] * 2))) / (klog * 50))) * ((TensionStartTop[i]) / log10(TensionStartTop[i] / (klog * 50)));
      }
      tension[i] = constrain(tension[i], 0, TensionRefS[i]);
    }
}

//----------------------------------------------
// ENCHAINEMENT DES PHASES ENTRE ELLES
//
//----------------------------------------------

void AUTOMATE() {
  for (int i = 0; i < VCF_VCA ; i++) {
    byte  mem1 = Etat_module[i];
    float mem2 = k_timer[i];
    byte  k = 1; // V3.01
    if (Choix_forme[i] == 1) {
      k = 2; // V3.01
    }

    //------------------------------------------------------------------------------

    if (Etat_module[i] == 0 && ((Etat_gate_in == 1) + (Choix_gate_in[i] == 1) > 0) ) {
      Etat_module[i] = 1;
    }
    else if (Etat_module[i] == 1 && Etat_gate_in == 0 && Choix_gate_in[i] == 0 ) {
      if (tension[i] > TensionRefS[i]) {
        Etat_module[i] = 2;
      } else {
        Etat_module[i] = 4;
      }
    }
    else if (Etat_module[i] == 1 && tension[i] >= 4095) {
      Etat_module[i] = 2;
    }
    else if (Etat_module[i] == 2 && (tension[i] <= TensionRefS[i] || k_timer[i]*tempsreel[i] > TimeRefD[i] / k )) {
      if (Etat_gate_in == 1) {
        Etat_module[i] = 3;
      } else {
        Etat_module[i] = 4;
      }
    }
    else if (Etat_module[i] == 2 && StartTop == 1) {
      Etat_module[i] = 1;
      StartTop = 0;
    }
    else if (Etat_module[i] == 3 && Etat_gate_in == 0) {
      Etat_module[i] = 4;
    }
    else if (Etat_module[i] == 4 && Etat_gate_in == 1 ) {
      Etat_module[i] = 1;
    }
    StartTop = 0;
    if (Etat_module[i] != mem1 || k_timer[i] != mem2) { // INITIALISATION DE L'INSTANT DE BASCULE ENTRE PHASE
      TimerStartTop[i] = micros();
      TensionStartTop[i] = constrain(tension[i], 0, 4095);
    }
  }
}

//----------------------------------------------
// CALCUL DE LA DUREE DE LA PHASE (ADR) ASSOCIEE AU POTENTIOMETRE
//----------------------------------------------
unsigned long RefTime(unsigned int potar) {
  for (int i = 0; i < VCF_VCA ; i++) {
    unsigned long k = int(potar * (DureeRef[Choix_timer[i]]));;
    k *= float(100000.00 / 1023.00);
    k += Offset[i] + 100;
    return k;
  }
}
// LECTURE PERIODIQUE DES INVERSEURS ET POTENTIOMETRES
//----------------------------------------------
void IHM()
{ for (int i = 0; i < VCF_VCA ; i++) {
    if (Choix_timer[i] == 1) {
      k_timer[i] = 1.00;
    }
    else {
      k_timer[i] = 0.05;
    }
    Offset[i] = int(DureeRef[Choix_timer[i]] * 500);

    TimeRefA[i]     = RefTime(POT[i][0]); TimeRefA[i] /= 2;
    TimeRefD[i]     = RefTime(POT[i][1]);
    TensionRefS[i]  = POT[i][2] * 4 ;
    TimeRefR[i]     = RefTime(POT[i][3]);
  }
}
