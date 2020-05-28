#include "mbed.h"
#include "TFC.h"

#define MidPointT 63

float ServoTurn;
int LineThreshold1;
int CA[128];

//Button Debouncer Engine
int ButtonPressed1 = 0;
int ButtonNotPressed1 = 0;
bool Button1 = false;
bool ButtonCounter1 = false;

//Engine ON/OFF
bool Engine = false;

//Button Debouncer Servo
int ButtonPressed0 = 0;
int ButtonNotPressed0 = 0;
bool Button0 = false;
bool ButtonCounter0 = false;

//Servo ON/OFF
bool Servo = false;

//USB connection
bool usb;

//Mid Point Memory
int MidPointMemory [50];
int MemoryIndex = 0;

int MidPointMemoryDifference;

//Tick Counter
int Tick;
int TickMemory;
bool ReadTick = true;
int TickDifference;

//Initial Case
int Switch_Position = 11;

//PD
int Derivative = 0;
int LastError;
float Kp = 0.9;
float Kd = 0.1;

//Automatic Threshold
bool AutomaticThreshold;
int HighValues;
int LowValues;
int Constant = -100;


//Functions
void SwitchToCase11()
{
    ReadTick = true;
    TickDifference = 0;
    TFC_BAT_LED2_OFF;
    Switch_Position = 11;
}

int main() {
    TFC_Init();
    char CameraArrayChar[128] = {' '};
    int LeftBorder = 0;
    int RightBorder = 0;
    int MidPoint = 0;
    uint32_t i = 0;
    uint32_t j = 0;
    float Error= 0;


   
    for(;;)
    {
        //read values from the potentiometers
        float ReadPot1 = TFC_ReadPot(1);
        float ReadPotRange = TFC_ReadPotRange(0);
        
        //Manual Read Threshold Value
        if (TFC_DIP_SWITCH_1_ON)
        {
            AutomaticThreshold = false;
        }
        else //Automatic Dynamic Threshold Search
        {
            AutomaticThreshold = true;
        }
        
        //Button Debouncer Servo
        if(TFC_PUSH_BUTTON_0_PRESSED & Button0 == false) 
        { //increment when button is pressed
            ButtonPressed0++; 
            if (ButtonPressed0 == 4000)
                {
                    Button0 = true; //flag that button is pressed
                    ButtonPressed0 = 0; //reset the counter
                }
        }
            
        if (Button0 == true) 
        { 
            ButtonNotPressed0++; //increment when button is not pressed
            if (ButtonNotPressed0 == 4000)
            {
        //ButtonCounter0 diffrentiates if the servo should be turned on or off
                if (ButtonCounter0 == false)
                { 
                    TFC_BAT_LED3_ON; //turn the LED on
                    Servo = true; //turn the servo on
                    ButtonCounter0 = true; //reset the flag (servo is on)
                }
                else
                {
                    TFC_BAT_LED3_OFF; //turn the LED off
                    Servo = false; //turn the servo off
                    ButtonCounter0 = false; //reset the flag (servo is off)
                }
                    Button0 = false; //resets the button press flag
                    ButtonNotPressed0 = 0; //reset the counter (button not pressed)
            }
        }
            
        ///Button Debouncer Engine
        if(TFC_PUSH_BUTTON_1_PRESSED & Button1 == false) 
        {
            ButtonPressed1++;
            if (ButtonPressed1 == 4000)
            {
                Button1 = true;
                ButtonPressed1 = 0;
            }
        }
            
        if (Button1 == true)
        {
            ButtonNotPressed1++;
            if (ButtonNotPressed1 == 4000)
            {
                if (ButtonCounter1 == false)
                {
                    TFC_BAT_LED0_ON;
                    TFC_HBRIDGE_ENABLE;   //Enable H-Bridge
                    Engine = true;
                    ButtonCounter1 = true;
                }
                else
                {
                    TFC_BAT_LED0_OFF;
                    TFC_HBRIDGE_DISABLE; //Disable H-Bridge
                    Engine = false;
                    ButtonCounter1 = false;
                }
                Button1 = false;
                ButtonNotPressed1 = 0;
            }
        }
        
        //USB Connection
        if (TFC_DIP_SWITCH_0_ON)
        {
            usb = true;
        }
        else
        {
            usb = false;    
        }
        
        if (usb == true)
        {
        Serial pc(USBTX, USBRX);    
        }
        
        
        switch(Switch_Position)
        {
        default:
        
   
        case 11:       
        
        //Set the speed of the motors using a potentiometer
        TFC_SetMotorPWM(ReadPot1,ReadPot1);
        
        if (TFC_LineScanImageReady > 0) //check if flag is non-zero 
        {
            TFC_LineScanImageReady = 0; //reset the flag back to zero
            //pc.printf("Threshold = %d.\r\n", LineThreshold1);
            
            if (AutomaticThreshold == true) //enable adaptive thresholding
            {
                for(i = 0; i < 128; i++)
                {
                    //store the output values in the array CA => CameraArray
                    CA[i] = ((int)TFC_LineScanImage0[i]);
                }
                
                //Segregate the array in the ascending order
                for (i = 0; i < 128; i++) 
                {
                    for (j = i + 1; j < 128; j++)
                    {
                        if (CA[i] > CA[j]) 
                        {
                            int a =  CA[i];
                            CA[i] = CA[j];
                            CA[j] = a;
                        } 
                    }
                }
                //Calculate threshold using smallest and largest numbers (ignoring the extreme ones)
                LowValues = ((CA[7] + CA[8] + CA[9] + CA[10] + CA[11])/5);
                HighValues = ((CA[120] + CA[119] + CA[118] + CA[117] + CA[116])/5);
                LineThreshold1 = ((LowValues + HighValues)/2) + Constant;
            }
            else //use manual thresholding
            {
            LineThreshold1 = (ReadPotRange);
            }

            //BORDER SEARCH
            for(i = 0; i < 128; i++)
            {
                //check if the ADC values from the camera are higher than the threshold
                if ((int)TFC_LineScanImage0[i] >= LineThreshold1) 
                {
                //if the value is higher, put the empty space in the string
                    CameraArrayChar[i] = 219; //white
                }
                else
                //otherwise put a black square
                {
                    CameraArrayChar[i] = ' '; //black
                }
                

                
                if (usb == true)
                {
                    printf("%c", CameraArrayChar[i]);
                }
            }
        }

            
            for(i = 63; i > 0; i--) //check from middle to the left
            {
                if (CameraArrayChar[i] == ' ') //if there's a black square (line)
                {
                    LeftBorder = i; //save it as a left border
                    break; 
                }
                else LeftBorder = 0; //if black square is not found, left border is equal to 0 
            }
            
            for(i = 65; i < 128; i++) //check from middle to the right
            {
                if (CameraArrayChar[i] == ' ') //if there's a black square (line)
                {
                    RightBorder = i; //save it as right border
                    break;
                }
                else RightBorder = 127; //if there's no black, save it as 127
            }
            //calculate mid point
            MidPoint = (int)(LeftBorder + (RightBorder - LeftBorder)/2 );
            
            //Mid Point Memory
            if (MemoryIndex <= 49) //after start-up begin by putting the midpoint values 
            //into an empty array [MidPointMemory]
            {
                MidPointMemory[MemoryIndex] = MidPoint;
                MemoryIndex++;
            }
            
            if (MemoryIndex == 50) //when the array is full, get rid of the oldest value,
            { // move all of the new ones by one address 
                for (int k = 49; k >= 0; k--)
                {
                    MidPointMemory[k]=MidPointMemory[k-1];
                } 
                //and put a new one in the "empty" address
                MidPointMemory[0] = MidPoint;
            }
            
            
            if (CameraArrayChar[63] == ' ' & Servo == true)
            {
                Switch_Position = 12;
            }

            //PD / P - enable/disable
            if (TFC_DIP_SWITCH_2_ON)
            {
                TFC_BAT_LED1_ON;
                LastError = Error;
                Error = (MidPointT-MidPoint);
                
                if (Engine == true)
                {
                    Derivative = Error - LastError;
                }
            
                ServoTurn = ((Kp * Error) + (Kd * Derivative))/-20;
                
            }
            else
            {
                TFC_BAT_LED1_OFF;
                Error = (MidPointT-MidPoint);
                ServoTurn = (Error/-20);
            }
            
            if (ServoTurn > 1)
            {
                ServoTurn = 1;
            }
            else if (ServoTurn < -1)
            {
                ServoTurn = -1;
            }
            
        if (usb == true)
        {
            
            printf("Left Border: %d ", LeftBorder);
            printf("Right Border: %d ", RightBorder);
            printf("Mid Point: %d ", MidPoint);
            printf("Error: %f ", Error);
            printf("Derivative: %d ", Derivative);
            printf("ServoTurn: %f ", ServoTurn);
            printf("Threshold = %d.\r\n", LineThreshold1);
            /*
            for(int i = 0; i < 50; i++) 
            {
            printf("%d ", MidPointMemory[i]);
            }
            */
        }
        //Servo Enable
         if (Servo == true)
         {
             TFC_SetServo(0,ServoTurn);
         }
         else
         {
             TFC_SetServo(0,0);
         }

        
        Tick++;

        break;
        

        case 12:
        if (usb == true)
        {
        printf("State 12  ");
        printf("ServoTurn: %f.\r\n", ServoTurn);
        }

        TFC_BAT_LED2_ON; //turn LED2 on
        if (ReadTick == true) //start measuring time
        {
            TickMemory = Tick;
            ReadTick = false;
        }
        
        //slow down and turn left
        if (MidPointMemory[15] <= 63) //read MidPoint from memory
        //depending on the value of the remembered value, decide to turn left or right
        {
            TFC_SetMotorPWM(ReadPot1-0.1,ReadPot1-0.1); //slow down
            ServoTurn = (-0.9); //turn left
        }
        //otherwise slow down and turn right
        else if (MidPointMemory[15] > 63) //read MidPoint from memory
        {
            TFC_SetMotorPWM(ReadPot1-0.1,ReadPot1-0.1); //slow down
            ServoTurn = (0.9); //turn right
        }
        TFC_SetServo(0,ServoTurn);
        Tick++; //increment time measuring Tick
        TickDifference = Tick - TickMemory; //calculate the time since the emergency algorithm
                                            //start up
        //Stop when time runs out
        if (TickDifference == 10000)
        {
            SwitchToCase11();
        }
        break;
        
        case 13:        

        break;
        
        case 14:    
 
        break;
        
        case 15:

        
        break;
        }   
    }
}
