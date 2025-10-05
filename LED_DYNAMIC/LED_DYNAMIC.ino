#define COL_BK  0    // 000 BLACK
#define COL_BL  1    // 001 BLUE
#define COL_RD  2    // 010 RED
#define COL_PR  3    // 011 PURPLE
#define COL_GR  4    // 100 GREEN
#define COL_SB  5    // 101 SKY BLUE
#define COL_YL  6    // 110 YELLOW
#define COL_WH  7    // 111 WHITE

#define DOTSUU          30

#define LED_PIN 32

#define LED_L1  5  
#define LED_L2  18
#define LED_L3  23
#define LED_L4  4
#define LED_L5  15
#define LED_L6  19

#define LED_H1  33
#define LED_H2  25
#define LED_H3  14
#define LED_H4  26
#define LED_H5  27

#define LED_R   2
#define LED_G   13
#define LED_B   12

#define LED_PERIOD  (100L)

void led_blink_int(void);
void log_print(void *arg);

volatile uint8_t on_led[DOTSUU]; 
volatile uint32_t del_micros ; 

void LED_proc_sq(int lnum,int nc)
{

  digitalWrite(LED_L1, LOW );
  digitalWrite(LED_L2, LOW );
  digitalWrite(LED_L3, LOW );
  digitalWrite(LED_L4, LOW );
  digitalWrite(LED_L5, LOW );
  digitalWrite(LED_L6, LOW );
  
  digitalWrite(LED_H1, LOW );
  digitalWrite(LED_H2, LOW );
  digitalWrite(LED_H3, LOW );
  digitalWrite(LED_H4, LOW );
  digitalWrite(LED_H5, LOW );

  digitalWrite(LED_R, HIGH);
  digitalWrite(LED_G, HIGH);
  digitalWrite(LED_B, HIGH);
 
  ets_delay_us(10);
  switch(lnum){
    case 1 : digitalWrite(LED_H1, HIGH);digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, LOW );digitalWrite(LED_L1, HIGH);digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, LOW );break; 
    case 2 : digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, HIGH);digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, LOW );digitalWrite(LED_L1, HIGH);digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, LOW );break; 
    case 3 : digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, HIGH);digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, LOW );digitalWrite(LED_L1, HIGH);digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, LOW );break; 
    case 4 : digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, HIGH);digitalWrite(LED_H5, LOW );digitalWrite(LED_L1, HIGH);digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, LOW );break; 
    case 5 : digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, HIGH);digitalWrite(LED_L1, HIGH);digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, LOW );break; 

    case 6 : digitalWrite(LED_H1, HIGH);digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, LOW );digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, HIGH);digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, LOW );break; 
    case 7 : digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, HIGH);digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, LOW );digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, HIGH);digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, LOW );break; 
    case 8 : digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, HIGH);digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, LOW );digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, HIGH);digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, LOW );break; 
    case 9 : digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, HIGH);digitalWrite(LED_H5, LOW );digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, HIGH);digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, LOW );break; 
    case 10: digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, HIGH);digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, HIGH);digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, LOW );break; 
            
    case 11: digitalWrite(LED_H1, HIGH);digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, LOW );digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, HIGH);digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, LOW );break; 
    case 12: digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, HIGH);digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, LOW );digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, HIGH);digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, LOW );break; 
    case 13: digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, HIGH);digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, LOW );digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, HIGH);digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, LOW );break; 
    case 14: digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, HIGH);digitalWrite(LED_H5, LOW );digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, HIGH);digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, LOW );break; 
    case 15: digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, HIGH);digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, HIGH);digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, LOW );break; 

    case 16: digitalWrite(LED_H1, HIGH);digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, LOW );digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, HIGH);digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, LOW );break; 
    case 17: digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, HIGH);digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, LOW );digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, HIGH);digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, LOW );break; 
    case 18: digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, HIGH);digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, LOW );digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, HIGH);digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, LOW );break; 
    case 19: digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, HIGH);digitalWrite(LED_H5, LOW );digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, HIGH);digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, LOW );break; 
    case 20: digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, HIGH);digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, HIGH);digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, LOW );break; 

    case 21: digitalWrite(LED_H1, HIGH);digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, LOW );digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, HIGH);digitalWrite(LED_L6, LOW );break; 
    case 22: digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, HIGH);digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, LOW );digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, HIGH);digitalWrite(LED_L6, LOW );break; 
    case 23: digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, HIGH);digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, LOW );digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, HIGH);digitalWrite(LED_L6, LOW );break; 
    case 24: digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, HIGH);digitalWrite(LED_H5, LOW );digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, HIGH);digitalWrite(LED_L6, LOW );break; 
    case 25: digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, HIGH);digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, HIGH);digitalWrite(LED_L6, LOW );break; 

    case 26: digitalWrite(LED_H1, HIGH);digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, LOW );digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, HIGH);break; 
    case 27: digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, HIGH);digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, LOW );digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, HIGH);break; 
    case 28: digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, HIGH);digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, LOW );digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, HIGH);break; 
    case 29: digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, HIGH);digitalWrite(LED_H5, LOW );digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, HIGH);break; 
    case 30: digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, HIGH);digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, HIGH);break; 

    default: digitalWrite(LED_H1, LOW );digitalWrite(LED_H2, LOW );digitalWrite(LED_H3, LOW );digitalWrite(LED_H4, LOW );digitalWrite(LED_H5, LOW );digitalWrite(LED_L1, LOW );digitalWrite(LED_L2, LOW );digitalWrite(LED_L3, LOW );digitalWrite(LED_L4, LOW );digitalWrite(LED_L5, LOW );digitalWrite(LED_L6, LOW ); 
  }

  ets_delay_us(5);

  switch(nc){
    case COL_BK: digitalWrite(LED_R, HIGH);digitalWrite(LED_G, HIGH);digitalWrite(LED_B, HIGH);break; 
    case COL_RD: digitalWrite(LED_R, LOW );digitalWrite(LED_G, HIGH);digitalWrite(LED_B, HIGH);break; 
    case COL_YL: digitalWrite(LED_R, LOW );digitalWrite(LED_G, LOW );digitalWrite(LED_B, HIGH);break; 
    case COL_GR: digitalWrite(LED_R, HIGH);digitalWrite(LED_G, LOW );digitalWrite(LED_B, HIGH);break; 
    case COL_SB: digitalWrite(LED_R, HIGH);digitalWrite(LED_G, LOW );digitalWrite(LED_B, LOW );break; 
    case COL_WH: digitalWrite(LED_R, LOW );digitalWrite(LED_G, LOW );digitalWrite(LED_B, LOW );break; 
    case COL_PR: digitalWrite(LED_R, LOW );digitalWrite(LED_G, HIGH);digitalWrite(LED_B, LOW );break; 
    case COL_BL: digitalWrite(LED_R, HIGH);digitalWrite(LED_G, HIGH);digitalWrite(LED_B, LOW );break; 
    default:digitalWrite(LED_R, HIGH);digitalWrite(LED_G, HIGH);digitalWrite(LED_B, HIGH);
  }  

}

void led_blink_int(void *arg)
{
  int i ; 
  
  while(1){  
    for(i=0;i<DOTSUU;i++){
      LED_proc_sq(i+1,on_led[i]);   
    }
  }
}


void setup() 
{
  int i ; 
  
  Serial.begin(115200);
  Serial.println("\n LED Demo");

  for(i=0;i<DOTSUU;i++){
      on_led[i] = 0 ; 
  }
  on_led[0] = COL_BL ;

  pinMode(LED_PIN, OUTPUT);

  pinMode(LED_L1, OUTPUT);
  pinMode(LED_L2, OUTPUT);
  pinMode(LED_L3, OUTPUT);
  pinMode(LED_L4, OUTPUT);
  pinMode(LED_L5, OUTPUT);
  pinMode(LED_L6, OUTPUT);
  
  pinMode(LED_H1, OUTPUT);
  pinMode(LED_H2, OUTPUT);
  pinMode(LED_H3, OUTPUT);
  pinMode(LED_H4, OUTPUT);
  pinMode(LED_H5, OUTPUT);

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  
  // ウォッチドッグ停止
  disableCore0WDT();
  disableCore1WDT(); 
  xTaskCreatePinnedToCore(  led_blink_int, "LED_TASK", 4096, NULL, 1, NULL, 0);
}

void loop() {
  int i ;

  int col, pos ; 
  int ctime = millis() / 1000;
  for(i=0;i<DOTSUU;i++)
    on_led[i] = 0 ;
  on_led[(ctime%90)/3] = ((ctime%90)%3==0)?COL_RD: (((ctime%90)%3==1)?COL_GR:COL_BL) ;
  
  Serial.print(" time = "); 
  Serial.print(millis(),DEC); 
  Serial.print(" on_led ="); 
  for(i=0;i<DOTSUU;i++)
    Serial.print(on_led[i],DEC);

  Serial.println();
    
  delay(100);
}
