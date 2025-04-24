/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "arm_math_types_f16.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// ADC and DAC configuration
#define ADC_SAMPLE_RATE 96000    // Sampling frequency in Hz
#define ADC_BUFFER_SIZE 64       // Size of ADC buffer for DMA transfer
#define SINE_TABLE_SIZE 8      // Size of sine wave lookup table
#define DAC_BUFFER_SIZE ADC_BUFFER_SIZE
#define BLOCK_SIZE 32            // Block size for filter processing
#define NUM_STAGE_IIR 16
#define NUM_STD_COEFFS 5
#define IIR_ORDER (NUM_STAGE_IIR*2)            // Order of the IIR filter (32 + 1 for index 0)
#define FIR_ORDER 455
#define NUM_BLOCKS (DAC_BUFFER_SIZE / BLOCK_SIZE)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

TIM_HandleTypeDef htim8;

/* USER CODE BEGIN PV */
uint16_t sine_wave[64] = {
 2047, 2248, 2447, 2642, 2831, 3013, 3185, 3346,
 3495, 3630, 3750, 3853, 3939, 4006, 4054, 4082,
 4095, 4088, 4061, 4014, 3948, 3864, 3762, 3644,
 3511, 3363, 3203, 3032, 2852, 2664, 2471, 2274,
 2075, 1875, 1678, 1485, 1296, 1114,  943,  782,
  633,  498,  378,  275,  189,  122,   74,   46,
   33,   40,   67,  114,  180,  264,  367,  485,
  618,  766,  926, 1097, 1279, 1472, 1666, 1861
};

uint32_t value_adc;
uint32_t value_dac=0;
// DMA buffers for ADC and DAC
uint16_t adcBuffer[ADC_BUFFER_SIZE];
uint16_t dacBuffer[DAC_BUFFER_SIZE];
// Signal processing buffers
float32_t sineTable[SINE_TABLE_SIZE];     // Carrier wave lookup table
float32_t adcValue[ADC_BUFFER_SIZE];      // ADC samples converted to float
float32_t modulatedSingal[DAC_BUFFER_SIZE]; // Buffer for modulated signal
float32_t *modulatedSignalPtr=&modulatedSingal[0];

// IIR filter buffers and structure
float32_t filt_out_BPF[ADC_BUFFER_SIZE];  // Output buffer for filtered signal
float32_t *filt_out_BPF_ptr = &filt_out_BPF[0];

// IIR filter structure and state variables
arm_biquad_cascade_df2T_instance_f32 iir_filter_BPF;
arm_fir_instance_f32 fir_filter_BPF;
float32_t iir_State_BPF[IIR_ORDER];  // State buffer for IIR filter (2 states per stage)
float32_t fir_State_BPF[BLOCK_SIZE+FIR_ORDER-1];
// IIR filter coefficients for bandpass filter
// Fstop1 : 10000Hz
// Fpass1 : 10300Hz
// Fpass2 : 11700Hz
// Fstop2 : 12000Hz
// Astop1 = Astop2 = 40dB
// Apass  = 1dB
// Denominator coefficients (a) - represents the feedback part of the filter
static const float32_t a_coeffs[IIR_ORDER] = {
    1, -23.3433254617316, 270.458923271493, -2068.05077951067, 11711.9227186727,
    -52280.9402944496, 191203.373802611, -588036.150343295, 1549228.77829786,
    -3544487.26024351, 7114999.55150585, -12629096.804023, 19940196.4604793,
    -28131336.186653, 35577251.704648, -40424177.8370125, 41319828.7727523,
    -38010952.4767753, 31456289.2334764, -23387992.8902516, 15588325.8205582,
    -9283456.97370102, 4917899.62201564, -2303697.87299914, 946792.887857377,
    -337917.637907267, 103316.564961882, -26563.5229314584, 5595.49926794879,
    -929.053986421767, 114.248700906604, -9.27223577617348, 0.37350446875285
};

// Numerator coefficients (b) - represents the feedforward part of the filter
static const float32_t b_coeffs[IIR_ORDER] = {
	5.32193000539747e-22,0,-8.51508800863595e-21,0,6.38631600647696e-20,
	0,-2.98028080302258e-19,0,9.6859126098234e-19,0,
	-2.32461902635761e-18,0,4.26180154832229e-18,0,-6.08828792617471e-18,
	0,6.84932391694654e-18,0,-6.08828792617471e-18,0,4.26180154832229e-18,
	0,-2.32461902635761e-18,0,9.6859126098234e-19,0,-2.98028080302258e-19,
	0,6.38631600647696e-20,0,-8.51508800863595e-21,0,5.32193000539747e-22

};

// Combined coefficients array for biquad cascade implementation
// Format: [b0, b1, b2, a1, a2] for each biquad section
float32_t iir_coeffs[NUM_STAGE_IIR * NUM_STD_COEFFS];
float32_t fir_Coeffs[FIR_ORDER]={
		-0.0058619374957668,-0.00110548757268269,-6.65227558823023e-05,0.000472837314931857,0.00143681124826632,0.00112415238239334,0.000801911066640192,-0.000547690079561553,-0.0011323610300659,
		-0.0017380801179726,-0.000932230048922747,-0.000148723735046442,0.00128568578088863,0.00160639984276964,0.00163835684675762,0.000334518298504674,-0.000685958557758057,-0.00187608636328981,
		-0.00167190953144402,-0.00109131423846801,0.000506875868767951,0.00141906228307277,0.00207245745840485,0.00125954659199557,0.000247782467419609,-0.00129873873262891,-0.00177252577932734,
		-0.00176405169315214,-0.000491040881254183,0.000599541751217487,0.00173806866296516,0.00160926777475315,0.00106759817544077,-0.000322075837379294,-0.00112105331220665,-0.00166353000803708,
		-0.00102386638299055,-0.000281750723617551,0.000840735721783062,0.00112680594130871,0.00114315906155463,0.000321543696576603,-0.000226269843788243,-0.000850036733469674,-0.000669668833415091,
		-0.000486447692187471,0.000119353361576069,0.000217951146292954,0.000379228443964327,5.75212351618366e-05,8.46511129970577e-05,-3.09365504689144e-05,0.000290597019966663,0.000265497576998013,
		0.000297678484178433,-0.000239208968608673,-0.00058913812565957,-0.000999824481204441,-0.000646542791320551,-6.80948384784012e-05,0.000972206331435297,0.00144149164682086,0.00144843046258974,
		0.000405277167301006,-0.000798468286678718,-0.00198788841423652,-0.00204350545691326,-0.00123649717141539,0.000541606091730558,0.00201160660965538,0.00277919389113708,0.00195686311890767,
		0.00024782026557904,-0.0019146292255454,-0.00304413483855933,-0.00286612738444896,-0.00102203314402476,0.001255529273831,0.00317050065020374,0.00337469991942061,0.00204584119663469,
		-0.000521086847336258,-0.00271714544807866,-0.00374709318498899,-0.00275170530401786,-0.000530787377914569,0.00210934848642278,0.00353842355722041,0.00334477603415372,0.00135575241534541,
		-0.00111670010848015,-0.00310930173063769,-0.00337140881751947,-0.00210824694131035,0.000253456216796522,0.00223042376210951,0.00312697016178281,0.00232405182002504,0.000590812297289742,
		-0.0013831766022154,-0.00237386544658797,-0.00223250798054159,-0.000943012843026377,0.000489637933612374,0.00155220324437664,0.00158125671139989,0.000965354391980629,-2.95932733799526e-05,
		-0.000609358482982824,-0.000760788389098633,-0.00038503642646027,-9.44170468044091e-05,2.70101413133411e-05,-0.000266271621220634,-0.00046350270231252,-0.000417957806839008,0.000232493240554885,
		0.00102303645670625,0.00161505666092839,0.00129298321975885,0.000163708583319281,-0.00149176468715826,-0.00260334378669089,-0.00257477728949065,-0.00100778600354249,0.00130608840875756,
		0.00336435863905496,0.0038170945642547,0.00236418886228624,-0.000603662426583025,-0.00349738963229766,-0.00491755581931147,-0.00381593669972745,-0.000711328943233871,0.00306778740137042,
		0.00544342674851751,0.00523339361974583,0.00225967816380871,-0.00193806016998797,-0.00538743059445196,-0.00615503192641872,-0.0038972506682691,0.000439750553099993,0.00456120281378936,
		0.00650052473181879,0.00513795503650621,0.00128045030616249,-0.00323763558857058,-0.00602321432339963,-0.00583101737860664,-0.00271971077843504,0.00155268201996887,0.00492543079390205,
		0.00566264022468019,0.00365838676897493,-1.57548871447771e-05,-0.00331657378782946,-0.00475121200090674,-0.00370939770151071,-0.00108897769482378,0.00169895059376438,0.00316116565573821,
		0.00289193594768946,0.00129585276058423,-0.000411913459659345,-0.00136831289902244,-0.00120164398588966,-0.000509769956258527,2.24546039276735e-06,-0.000248900317243912,-0.000936873684382396,
		-0.00136937745654684,-0.000691071570873844,0.00106806421934975,0.00313221156136082,0.00400031920062787,0.00270288564380062,-0.000735093251634656,-0.00470269948161292,-0.00701559692999087
		,-0.00581748988359823,-0.00112216985997711,0.00516570175538139,0.00970473108243918,0.00973707932007646,0.00444394903471499,-0.00398737992932515,-0.0114777291095177,-0.0137691213827231,
		-0.00905522857926266,0.00103412232218746,0.011640570673122,0.0172559565033163,0.0143397920564666,0.00369420678537223,-0.00984804401840161,-0.0193675930525277,-0.0196318669571992,
		-0.00972385202522174,0.00588131511321055,0.0195501271953583,0.0240135984410343,0.0164548084611113,-8.55484802585995e-06,-0.0173424865316796,-0.026753042656369,-0.022951454181482,
		-0.00731555302814406,0.0127577236183401,0.0271507285155341,0.0283631019716972,0.0152239726376496,-0.00603121969121167,-0.0249400970157627,-0.0317877835677022,-0.0228295031673886,
		-0.00212740948749779,0.0200821513483043,0.032687994354383,0.0290951911733298,0.0108920723533631,-0.0130501310704183,-0.0307250921186458,-0.0332566362724703,-0.0191816785606645,
		0.00451131953301463,0.0260809757836744,0.0346873171518404,0.0260809757836744,0.00451131953301463,-0.0191816785606645,-0.0332566362724703,-0.0307250921186458,-0.0130501310704183,
		0.0108920723533631,0.0290951911733298,0.032687994354383,0.0200821513483043,-0.00212740948749779,-0.0228295031673886,-0.0317877835677022,-0.0249400970157627,-0.00603121969121167,
		0.0152239726376496,0.0283631019716972,0.0271507285155341,0.0127577236183401,-0.00731555302814406,-0.022951454181482,-0.026753042656369,-0.0173424865316796,-8.55484802585995e-06,
		0.0164548084611113,0.0240135984410343,0.0195501271953583,0.00588131511321055,-0.00972385202522174,-0.0196318669571992,-0.0193675930525277,-0.00984804401840161,0.00369420678537223,
		0.0143397920564666,0.0172559565033163,0.011640570673122,0.00103412232218746,-0.00905522857926266,-0.0137691213827231,-0.0114777291095177,-0.00398737992932515,0.00444394903471499,
		0.00973707932007646,0.00970473108243918,0.00516570175538139,-0.00112216985997711,-0.00581748988359823,-0.00701559692999087,-0.00470269948161292,-0.000735093251634656,0.00270288564380062,
		0.00400031920062787,0.00313221156136082,0.00106806421934975,-0.000691071570873844,-0.00136937745654684,-0.000936873684382396,-0.000248900317243912,2.24546039276735e-06,-0.000509769956258527,
		-0.00120164398588966,-0.00136831289902244,-0.000411913459659345,0.00129585276058423,0.00289193594768946,0.00316116565573821,0.00169895059376438,-0.00108897769482378,-0.00370939770151071,
		-0.00475121200090674,-0.00331657378782946,-1.57548871447771e-05,0.00365838676897493,0.00566264022468019,0.00492543079390205,0.00155268201996887,-0.00271971077843504,-0.00583101737860664,
		-0.00602321432339963,-0.00323763558857058,0.00128045030616249,0.00513795503650621,0.00650052473181879,0.00456120281378936,0.000439750553099993,-0.0038972506682691,-0.00615503192641872,
		-0.00538743059445196,-0.00193806016998797,0.00225967816380871,0.00523339361974583,0.00544342674851751,0.00306778740137042,-0.000711328943233871,-0.00381593669972745,-0.00491755581931147,
		-0.00349738963229766,-0.000603662426583025,0.00236418886228624,0.0038170945642547,0.00336435863905496,0.00130608840875756,-0.00100778600354249,-0.00257477728949065,-0.00260334378669089,
		-0.00149176468715826,0.000163708583319281,0.00129298321975885,0.00161505666092839,0.00102303645670625,0.000232493240554885,-0.000417957806839008,-0.00046350270231252,-0.000266271621220634,
		2.70101413133411e-05,-9.44170468044091e-05,-0.00038503642646027,-0.000760788389098633,-0.000609358482982824,-2.95932733799526e-05,0.000965354391980629,0.00158125671139989,0.00155220324437664,
		0.000489637933612374,-0.000943012843026377,-0.00223250798054159,-0.00237386544658797,-0.0013831766022154,0.000590812297289742,0.00232405182002504,0.00312697016178281,0.00223042376210951,
		0.000253456216796522,-0.00210824694131035,-0.00337140881751947,-0.00310930173063769,-0.00111670010848015,0.00135575241534541,0.00334477603415372,0.00353842355722041,0.00210934848642278,
		-0.000530787377914569,-0.00275170530401786,-0.00374709318498899,-0.00271714544807866,-0.000521086847336258,0.00204584119663469,0.00337469991942061,0.00317050065020374,0.001255529273831,
		-0.00102203314402476,-0.00286612738444896,-0.00304413483855933,-0.0019146292255454,0.00024782026557904,0.00195686311890767,0.00277919389113708,0.00201160660965538,0.000541606091730558,
		-0.00123649717141539,-0.00204350545691326,-0.00198788841423652,-0.000798468286678718,0.000405277167301006,0.00144843046258974,0.00144149164682086,0.000972206331435297,-6.80948384784012e-05,
		-0.000646542791320551,-0.000999824481204441,-0.00058913812565957,-0.000239208968608673,0.000297678484178433,0.000265497576998013,0.000290597019966663,-3.09365504689144e-05,8.46511129970577e-05,
		5.75212351618366e-05,0.000379228443964327,0.000217951146292954,0.000119353361576069,-0.000486447692187471,-0.000669668833415091,-0.000850036733469674,-0.000226269843788243,0.000321543696576603,
		0.00114315906155463,0.00112680594130871,0.000840735721783062,-0.000281750723617551,-0.00102386638299055,-0.00166353000803708,-0.00112105331220665,-0.000322075837379294,0.00106759817544077,
		0.00160926777475315,0.00173806866296516,0.000599541751217487,-0.000491040881254183,-0.00176405169315214,-0.00177252577932734,-0.00129873873262891,0.000247782467419609,0.00125954659199557,
		0.00207245745840485,0.00141906228307277,0.000506875868767951,-0.00109131423846801,-0.00167190953144402,-0.00187608636328981,-0.000685958557758057,0.000334518298504674,0.00163835684675762,
		0.00160639984276964,0.00128568578088863,-0.000148723735046442,-0.000932230048922747,-0.0017380801179726,-0.0011323610300659,-0.000547690079561553,0.000801911066640192,0.00112415238239334,
		0.00143681124826632,0.000472837314931857,-6.65227558823023e-05,-0.00110548757268269,-0.0058619374957668
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */
float32_t dc=1800.0f;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief  Callback function for ADC conversion half complete
 * @param  hadc: ADC handle pointer
 * @note   Processes first half of ADC buffer:
 *         1. Converts ADC samples to float
 *         2. Modulates with carrier wave
 *         3. Applies IIR filter
 *         4. Outputs to DAC with DC offset
 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc){
	uint32_t sineIndex=0;
	for(uint32_t i=0;i<ADC_BUFFER_SIZE/2;i++){
		adcValue[i]=(float32_t)(adcBuffer[i]);
		modulatedSingal[i]=adcValue[i]*sineTable[sineIndex];
		sineIndex=(sineIndex+1)%SINE_TABLE_SIZE;
	}
	//arm_fir_f32(&fir_filter_BPF, modulatedSignalPtr+BLOCK_SIZE,filt_out_BPF_ptr+BLOCK_SIZE , BLOCK_SIZE);
	arm_biquad_cascade_df2T_f32(&iir_filter_BPF, modulatedSignalPtr, filt_out_BPF_ptr, BLOCK_SIZE);

	for(int i=0;i<DAC_BUFFER_SIZE/2;i++){
		dacBuffer[i]=(uint32_t)(filt_out_BPF[i]+dc);
	}
}

/**
 * @brief  Callback function for ADC conversion complete
 * @param  hadc: ADC handle pointer
 * @note   Processes second half of ADC buffer while DMA fills first half
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
	uint32_t sineIndex=0;
	for(uint32_t i=ADC_BUFFER_SIZE/2;i<ADC_BUFFER_SIZE;i++){
		adcValue[i]=(float32_t)(adcBuffer[i]);
		modulatedSingal[i]=adcValue[i]*sineTable[sineIndex];
		sineIndex=(sineIndex+1)%SINE_TABLE_SIZE;
	}
	//arm_fir_f32(&fir_filter_BPF, modulatedSignalPtr+BLOCK_SIZE,filt_out_BPF_ptr+BLOCK_SIZE , BLOCK_SIZE);
	arm_biquad_cascade_df2T_f32(&iir_filter_BPF, modulatedSignalPtr+BLOCK_SIZE, filt_out_BPF_ptr+BLOCK_SIZE, BLOCK_SIZE);
	for(int i=DAC_BUFFER_SIZE/2;i<DAC_BUFFER_SIZE;i++){
		dacBuffer[i]=(uint32_t)(filt_out_BPF[i]+dc);
	}
}

/**
 * @brief  Generates sine wave lookup table for carrier signal
 * @note   Creates one complete cycle of sine wave with SINE_TABLE_SIZE points
 */
void GenerateSineWave(void){
	for(int i=0; i<SINE_TABLE_SIZE;i++){
//		sineTable[i]=arm_sin_q15(2.0f*PI*i/SINE_TABLE_SIZE);
		sineTable[i] = 3.4028235e+38;
	}
}

/**
 * @brief  Initializes the IIR filter
 * @note   Converts filter coefficients to biquad cascade form
 *         Each biquad section requires 5 coefficients: b0, b1, b2, a1, a2
 *         The filter is implemented as a cascade of second-order sections
 *         Direct Form II Transposed structure is used for better numerical stability
 */
void InitializeIIRFilter(void) {
    // Convert coefficients to biquad cascade form
     for(int i = 0; i < NUM_STAGE_IIR; i++) {
        // Get coefficients for this biquad section
        float32_t b0 = b_coeffs[i*2];
        float32_t b1 = b_coeffs[i*2 + 1];
        float32_t b2 = b_coeffs[i*2 + 2];
        float32_t a1 = a_coeffs[i*2 + 1];
        float32_t a2 = a_coeffs[i*2 + 2];
        
        // Normalize coefficients by a0 (which is 1)
        iir_coeffs[i*5] = b0;      // b0
        iir_coeffs[i*5 + 1] = b1;  // b1
        iir_coeffs[i*5 + 2] = b2;  // b2
        iir_coeffs[i*5 + 3] = a1; // -a1 (negative for Direct Form II Transposed)
        iir_coeffs[i*5 + 4] = a2; // -a2 (negative for Direct Form II Transposed)
    }
    
    // Initialize the IIR filter structure
    arm_biquad_cascade_df2T_init_f32(&iir_filter_BPF, NUM_STAGE_IIR, &iir_coeffs[0], &iir_State_BPF[0]);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DAC_Init();
  MX_ADC1_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  GenerateSineWave();
  //arm_fir_init_f32(&fir_filter_BPF, FIR_ORDER, &fir_Coeffs[0], &fir_State_BPF[0], BLOCK_SIZE);
  InitializeIIRFilter();
  HAL_TIM_Base_Start(&htim8);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adcBuffer, ADC_BUFFER_SIZE);
  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)(sine_wave), DAC_BUFFER_SIZE, DAC_ALIGN_12B_R);

  /* USER CODE END 2 */
	  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 4000);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T8_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 1000-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 1-1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
