/*  Luis Cupido - CT1DMk - 2025

	PLL setup for ADF4350 ADF4351 chip
	Uses an AT328P/arduino nano,
        	
    V 1.0 - port from syn1 with ADF4356 (ADF5356) into ADF4350/51

*/

#include <EEPROM.h>

// greetings
#define DMSG  "uPLO Ver 1.0, ADF4350/1  -  Luis Cupido, Feb.2025"

// freq range
#define MINFREQ  130000UL
#define MAXFREQ 4600000UL

// f0, default 2.500 GHz
#define  OFREQ		2500000UL // Output Freq. KHz

// PLL parameters
#define  RFREQ         25UL // Reference freq. MHz
#define  FCOMP  	   5000UL // Freq. comp. KHz
#define  CHSPACE      100UL // channel space in KHz, same as, freq resulution
#define  GCD_FC_CH	  100UL // greatest common divider of (fcomp,chspace)
#define  INTCLK       125UL // internal logic clock, 125kHz
#define  LD_TSH       90    // Lock detection threshold, 90 out of 100 reads, or 90%



// Register definitions for ADF4350/51

// N(30..15), Frac(14..3)
#define R0MSK  0x7FFFFFF8UL // mask bits  = 0111.1111.1111.1111.1111.1111.1111.1000
#define R0FIX  0x00000000UL // fixed bits = 0000.0000.0000.0000.0000.0000.0000.0000

// MOD(14..3), presc(27)
#define R1MSK  0x00007FF8UL // mask bits  = 0000.0000.0000.0000.0111.1111.1111.1000
#define R1FIX  0x00008001UL // fixed bits = 0000.0000.0000.0000.1000.0000.0000.0001

// spurs(30,29), mux(28..26), ref2x(25), ref/2(24), rdiv(23..14), db(13), cp(12..9), LDF(8), LDP(7), PDpol(6), Pdown(5), cp3s(4), crst(3)
  #define R2MSK  0x00FFB000UL // mask bits  = 0000.0000.1111.1111.1100.0000.0000.0000
  #define R2FIX  0x18000C42UL // fixed bits = 0001.1000.0000.0000.0000.1101.0100.0010 (for 2mA of PD)
//#define R2FIX  0x18001942UL // fixed bits = 0001.1000.0000.0000.0001.1001.0100.0010 (for 4mA of PD)

// resync functions and etc. Antibacklash(22)=0 for Frac, =1 for Int w/ better PN
#define R3MSK  0x000FF000UL // mask bits  = 0000.0000.0000.0000.0000.0000.0000.0000
#define R3FIX  0x00000003UL // fixed bits = 0000.0000.0000.0000.0000.0000.0000.0011

// Out divider(22..20), divclk(19..12), output enabled
#define R4MSK  0x007FF000UL // mask bits  = 0000.0000.0111.1111.1111.0000.0000.0000
#define R4FIX  0x0080023CUL // fixed bits = 0000.0000.1000.0000.0000.0010.0011.1100

// LD_mode(23,22), misc fixed
#define R5MSK  0x00000000UL // mask bits  = 0000.0000.0000.0000.0000.0000.0000.0000
#define R5FIX  0x00580005UL // fixed bits = 0000.0000.0101.1000.0000.0000.0000.0101



// Pinout assignements

//PLL SPI
#define  P_SCK    4
#define  P_SDA    5
#define  P_SLD    3
#define  P_MUX    6

#define  P_CEN    2
#define  LD_LED   13
#define  LD_OUT   7


// global vars

unsigned long freq, ref;
unsigned long	r0,r1,r2,r3,r4,r5;
int           lld;


// load and store from eeprom

unsigned long e2rd_lval();
void          e2st_lval(unsigned long, byte); 


// freq and PLL funcions

void  calc_freq(unsigned long, byte);
void  init_pll(void);
void  reg_pll(unsigned long);
int   get_lock(void);




void setup()
{
  Serial.begin(9600);			// start serial port
  
  // Digital Pin modes
  pinMode(P_SCK, OUTPUT);
  pinMode(P_SDA, OUTPUT);
  pinMode(P_SLD, OUTPUT);
  pinMode(P_CEN, OUTPUT);
  pinMode(LD_LED, OUTPUT);
  pinMode(LD_OUT, OUTPUT);
  
  pinMode(P_MUX, INPUT);      // pin is digital input //analogReference(DEFAULT); // using VCC as reference
  
  // Init SPI Lines
  digitalWrite (P_SCK,LOW);
  digitalWrite (P_SDA,LOW);
  digitalWrite (P_SLD,LOW);

  // Enable PLL
  digitalWrite (P_CEN,HIGH);


  // wait for power up voltages to settle
  delay(200);

  Serial.println(DMSG);

  // get freq from eeprom
  freq = e2rd_lval();
  
  if ((freq < MINFREQ)||(freq>MAXFREQ))
  {
    freq = OFREQ;    // if not valid load default.
    ref = RFREQ;
    Serial.print("Read eeprom FAIL... default to ");
  }
  else  Serial.print("Read eeprom OK...  ");

  Serial.print("Freq = ");
  Serial.println(freq);
  Serial.println("");

  // setup frequency
  calc_freq(freq, ref);
  init_pll();
  
  lld=1;
}


void loop()
 {
   unsigned long lval;
   char cc;
   
  
  // lock detection and led blink when unlock
   
   lval = get_lock();       // Serial.println(lval); // lock value debug
   
   if (lval < LD_TSH) {   lld=-lld;
                          //digitalWrite(LD_OUT,LOW);     // pin as LOCK-DETECTOR: open collector, active low on locked condition
                          digitalWrite(LD_OUT,HIGH);    // pin as ALARM: open collector, active low on unlocked lock
                          }
                          
   else               {   lld=1;
                          //digitalWrite(LD_OUT,HIGH);    // pin as LOCK-DETECTOR: open collector, active low on locked condition
                          digitalWrite(LD_OUT,LOW);     // pin as ALARM: open collector, active low on unlocked lock
                          }
   
   if (lld==1) digitalWrite(LD_LED,HIGH);
   else        digitalWrite(LD_LED,LOW);



  // check serial commands

  if ( Serial.available() )
    {
      cc = Serial.read();


      if ( (cc=='R')||(cc=='r') )
      {
        lval = Serial.parseInt();
        
        if ( (lval != 10) && (lval != 25) && (lval != 100) )
        {
          Serial.println("Ref. out of range...");
          Serial.println("");
        }
        else
        {
          ref = (byte)lval;
          
          //Serial.println(DMSG);
          //Serial.print("Ref = ");
          //Serial.println(ref);
          //Serial.println("");

          calc_freq(freq, ref);
          init_pll();
        }  
      }
 

      if (cc=='F' or cc=='f')
      {
        lval = Serial.parseInt();
        
        if ( (lval > MAXFREQ) || (lval < MINFREQ))
        {
          Serial.println("Freq. out of range...");
          Serial.println("");
        }
        else
        {
          freq = lval;

          //Serial.println(DMSG);
          //Serial.print("Freq = ");
          //Serial.println(freq);
          //Serial.println("");

          calc_freq(freq, ref);
          init_pll();
        }  
      }


      if ( (cc=='P')||(cc=='p') )
      {
        Serial.println("Freq. stored on e2p...");
        Serial.print("Freq = ");
        Serial.println(freq);
        Serial.print("Ref  = ");
        Serial.println(ref);
        Serial.println();
        e2st_lval(freq, ref);
      }

      if ( (cc=='S')||(cc=='s') )
      {
        Serial.print("Freq = ");
        Serial.println(freq);
        Serial.print("Ref  = ");
        Serial.println(ref);
        Serial.print("Lock = ");
        Serial.println(get_lock());
        Serial.println();
      }

      if ((cc=='H')||(cc=='h')||(cc=='?'))
      {
        Serial.println(DMSG);
        Serial.println();
        Serial.println("Commands:");
        Serial.println("          H <enter>             This help");
        Serial.println("          S <enter>             Show status");
        Serial.println("          F xxxxxxx<enter>      Set frequency in KHz");
        Serial.println("          P <enter>             Store, make current settings the boot default");
        Serial.println();
        Serial.println("          R xxx<enter>          Select reference in MHz, 10, 25 or 100 only");
        Serial.println("                                requires a Store (P) and a reboot (OFF/ON)");
        Serial.println();
        Serial.println();
      }


    }

}



// Calculates from frequency value into the PLL registers
void calc_freq(unsigned long ffreq, byte ref)
{
  unsigned long fvco;
  unsigned long nint;
  unsigned long frac;
  unsigned long mod;
  unsigned long rdiv;
  unsigned long cdiv;
  
  unsigned long iref, odiv;
  unsigned long pre, rd2;


  fvco = ffreq;
  odiv=0;

  while (fvco < 2200000 ){ fvco = 2*fvco;
                           odiv ++;			}

  if ( fvco > 3000000 ) pre = 1;
  else                  pre = 0;

  if ( ref < 100 )    { iref = 2*ref; rd2 = 1; }
  else                { iref = ref;   rd2 = 0; }

  // pll parameters
  rdiv = (iref*1000) / FCOMP;
  mod  = FCOMP / GCD_FC_CH;
  nint = fvco / FCOMP;
  frac = (fvco % FCOMP) * mod / FCOMP;
  cdiv = FCOMP/INTCLK;


  // Compose PLL registers
  r0 = (((nint << 15)+(frac << 3)) & R0MSK) + R0FIX;
  r1 = (((mod << 3)+(pre << 27)) & R1MSK) + R1FIX;
  r2 = ((rdiv << 14) & R2MSK) + (rd2 << 25) + R2FIX;
  r3 = R3FIX;
  r4 = (((odiv << 20)+(cdiv << 12)) & R4MSK) + R4FIX;
  r5 = R5FIX;


  // reverse calculation, obtained frequency
  long long llfvco = (((long long)nint*(long long)mod + (long long)frac) * FCOMP)/(long long)mod;
  long ofvco = (long) llfvco;
  long ofreq = ofvco  >> odiv;

  // update global var freq
  freq = ofreq;   
  
  // some useful debug
  Serial.print("Requested Freq  = ");
  Serial.println(ffreq);
  Serial.print("Requested fVCO  = ");
  Serial.println(fvco);
  Serial.println();
  
  Serial.print("Calculated Nint = ");
  Serial.println(nint);
  Serial.print("Calculated Frac = ");
  Serial.println(frac);
  Serial.print("Fixed Modulus   = ");
  Serial.println(mod);
  Serial.print("Fixed Ref. div. = ");
  Serial.print(rdiv); if (ref < 100) Serial.print("  x2");
  Serial.println();

  Serial.println();

  Serial.print("Obtained fVCO   = ");
  Serial.println(ofvco);
  Serial.print("Obtained Freq   = ");
  Serial.println(ofreq);
  Serial.println("");


  // register values
  Serial.println("PLL register values loaded:");

  Serial.print("r0 = "); hex32prt(r0); Serial.print(" - "); bin32prt(r0); Serial.println();
  Serial.print("r1 = "); hex32prt(r1); Serial.print(" - "); bin32prt(r1); Serial.println();
  Serial.print("r2 = "); hex32prt(r2); Serial.print(" - "); bin32prt(r2); Serial.println();
  Serial.print("r3 = "); hex32prt(r3); Serial.print(" - "); bin32prt(r3); Serial.println();
  Serial.print("r4 = "); hex32prt(r4); Serial.print(" - "); bin32prt(r4); Serial.println();
  Serial.print("r5 = "); hex32prt(r5); Serial.print(" - "); bin32prt(r5); Serial.println();
  Serial.println();
  Serial.println();

}




//-------------------------------------
// Inits PLL, loads all registers

void init_pll(void)
{
  // general init
  reg_pll(r5);
  reg_pll(r4);
  reg_pll(r3);
  reg_pll(r2);
  reg_pll(r1);
  delay(10);
  reg_pll(r0);
  
}
  
	
//-------------------------------------
// Send one register to the pll chip
// register has 32 bit

void reg_pll(unsigned long pr)
{
int    j;

  //send bits MSB first
  for(j=31;j>=0;j--)
    {
      if((0x80000000UL & pr)==0)	digitalWrite(P_SDA, LOW);
      else			digitalWrite(P_SDA, HIGH);
		
      digitalWrite(P_SCK, HIGH );
      digitalWrite(P_SCK, LOW );
	  
      pr=pr<<1;		// shift gets next bit
    }

  // latch data
  digitalWrite(P_SLD, HIGH );
  digitalWrite(P_SLD, LOW );
  
  // rest with SDA low
  digitalWrite(P_SDA, LOW );
  
}



//-----------------------------------
// reads the lock det pin
// 100x read pin with 2ms delay, total time 0.2s

int get_lock()
  {
   int i, v;

   v=0;
   
   for (i=0; i<100; i++)
    { if ( digitalRead(P_MUX) == true ) v++ ;
      delay(2);   }
     
   return (v);
 }


//---------------------------------------
// EEPROM store and read

// store freq
void e2st_lval(unsigned long val, byte xref)
{

  EEPROM.write(0, byte(val&0x000000FF) );
  val >>= 8;
  EEPROM.write(1, byte(val&0x000000FF) );
  val >>= 8;
  EEPROM.write(2, byte(val&0x000000FF) );
  val >>= 8;
  EEPROM.write(3, byte(val&0x000000FF) );

  EEPROM.write(4, xref);
}


unsigned long e2rd_lval(void)
{
  unsigned long val;
  
  val  = (unsigned long)EEPROM.read(3);
  val <<=8;
  val |= (unsigned long)EEPROM.read(2);
  val <<=8;
  val |= (unsigned long)EEPROM.read(1);
  val <<=8;
  val |= (unsigned long)EEPROM.read(0);

  ref = EEPROM.read(4);

  return(val);
}


// hex32bit print
void hex32prt(unsigned long xx)
{
  byte bx;
  int i;
  
  Serial.print("0x");
  for (i=0; i<8; i++)
  {
    bx=(byte)((xx&0xF0000000)>>28);
    Serial.print(bx,HEX);
    xx = xx << 4; 
  }
}


// bin32bit print
void bin32prt(unsigned long xx)
{
  int i;
  
  for (i=0; i<32; i++)
  {
    if ( (xx&0x80000000)==0 ) Serial.print("0");
    else                      Serial.print("1");
    xx = xx << 1; 
  }
}
 
