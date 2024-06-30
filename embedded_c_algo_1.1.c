//#include <stdint.h>
#include <math.h>
#include "apollo3p.h"
//#include <core_cm4.h>
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

//*****************************************************************************
// Defines ( LEDs, mapping them to their respective GPIO numbers)
//*****************************************************************************
#define EVB_LED0        10
#define EVB_LED1        30
#define EVB_LED2        15
#define EVB_LED3        14
#define EVB_LED4        17

#define TARGET_CLASS 0 //0,1,-1(RT MODE)
#define OBS_DUR 20
#define PATCH_DUR 100-3
#define IGNORE1ST 0
#define PIR_SENST 0.1 //less is more sensitive digital PIR
#define MAX_FEATURES 6

#define ADC_SAMPLE_BUF_SIZE 128

//*****************************************************************************
// Define a circular buffer to hold the ADC samples
//*****************************************************************************
#define ADC_EXAMPLE_DEBUG   1
#define WAKE_INTERVAL_IN_MS     1000
#define XT_PERIOD               32768
#define WAKE_INTERVAL           XT_PERIOD * WAKE_INTERVAL_IN_MS * 1e-3

// Select an arbitrary page address in flash instance 1.
// 260KB = 0x41000.
#define ARB_PAGE_ADDRESS (AM_HAL_FLASH_INSTANCE_SIZE + (2 * AM_HAL_FLASH_PAGE_SIZE))
#define DS_SZ 510 //6 cols *85 records
static uint32_t dataset[DS_SZ];	

// ADC Sample buffer.
uint32_t g_ui32ADCSampleBuffer[ADC_SAMPLE_BUF_SIZE * 3];
am_hal_adc_sample_t SampleBuffer[ADC_SAMPLE_BUF_SIZE * 3];

// ADC Device Handle.
static void *g_ADCHandle;

// ADC DMA complete flag.
volatile bool                   g_bADCDMAComplete;

// ADC DMA error flag.
volatile bool                   g_bADCDMAError;

// Define the ADC SE0,1,2 pins to be used.
const am_hal_gpio_pincfg_t g_AM_PIN_16_ADCSE0 = { .uFuncSel = AM_HAL_PIN_16_ADCSE0 };
const am_hal_gpio_pincfg_t g_AM_PIN_29_ADCSE1 = { .uFuncSel = AM_HAL_PIN_29_ADCSE1 };
const am_hal_gpio_pincfg_t g_AM_PIN_11_ADCSE2 = { .uFuncSel = AM_HAL_PIN_11_ADCSE2 };

bool file_exists()
{
	// Check if file beginning is 0
	bool exists=false;
	am_util_stdio_printf("  ... verifying file existence.\n");
	for (short ix = 0; ix < 6; ix++ )
	{
		if (*(uint32_t*)(ARB_PAGE_ADDRESS + (ix*4)) != 4294967295 )
		{
			am_util_stdio_printf("Non-Empty Value = %u.\n",*(uint32_t*)(ARB_PAGE_ADDRESS + (ix * 4)) );
			exists=true;
			break;
		}
	}
	return exists;
}

void erase_flash_page(uint32_t ui32PageAddr)
{
	int32_t i32ReturnCode;
	int32_t i32ErrorFlag = 0;
	// Erase the whole block of FLASH instance 1.
	am_util_stdio_printf("  ... erasing all of flash instance %d.\n", AM_HAL_FLASH_ADDR2INST(ARB_PAGE_ADDRESS) );
	i32ReturnCode = am_hal_flash_mass_erase(AM_HAL_FLASH_PROGRAM_KEY, 1);
	// Check for an error from the HAL.
	if (i32ReturnCode)
	{
		am_util_stdio_printf("FLASH_MASS_ERASE i32ReturnCode =  0x%x.\n",i32ReturnCode);
		i32ErrorFlag++;
	}
}
void read_ds()
{
  short ctr=0;
	am_util_stdio_printf("Reading DS...\n");

	for (short ix = 0; ix < DS_SZ; ix++)
	{
		dataset[ix]=*(uint32_t*)(ARB_PAGE_ADDRESS + (ix*4));
		am_util_stdio_printf("%u ",dataset[ix]);
		if (ctr > 6){
			ctr=0;
			am_util_stdio_printf("\n");
		}
		ctr++;
	}
}
void create_file()
{
    int32_t i32ReturnCode;
    int32_t i32ErrorFlag = 0;
    uint32_t *pui32Dst;
	// Program a few words in a page in the main block of instance 1.
    pui32Dst = (uint32_t *) ARB_PAGE_ADDRESS;
    i32ReturnCode = am_hal_flash_program_main(AM_HAL_FLASH_PROGRAM_KEY,
                                              dataset,
                                              pui32Dst,
                                              DS_SZ);
    // Check for an error from the HAL.
    if (i32ReturnCode)
    {
        am_util_stdio_printf("FLASH program page at 0x%08x "
                             "i32ReturnCode = 0x%x.\n",
                             ARB_PAGE_ADDRESS,
                             i32ReturnCode);
        i32ErrorFlag++;
    }
}
// Init function for Timer A0.
void stimer_init(void)
{
    // Enable compare A interrupt in STIMER
    am_hal_stimer_int_enable(AM_HAL_STIMER_INT_COMPAREA);

    // Enable the timer interrupt in the NVIC.
    NVIC_EnableIRQ(STIMER_CMPR0_IRQn);

    // Configure the STIMER and run
    am_hal_stimer_config(AM_HAL_STIMER_CFG_CLEAR | AM_HAL_STIMER_CFG_FREEZE);
    am_hal_stimer_compare_delta_set(0, WAKE_INTERVAL);
    am_hal_stimer_config(AM_HAL_STIMER_XTAL_32KHZ | AM_HAL_STIMER_CFG_COMPARE_A_ENABLE);
}
//*****************************************************************************
// Timer Interrupt Service Routine (ISR)
//*****************************************************************************
void am_stimer_cmpr0_isr(void)
{
    // Check the timer interrupt status.
    am_hal_stimer_int_clear(AM_HAL_STIMER_INT_COMPAREA);
    am_hal_stimer_compare_delta_set(0, WAKE_INTERVAL);
}
//*****************************************************************************
// Interrupt handler for the ADC.
//*****************************************************************************
void am_adc_isr(void)
{
    uint32_t ui32IntMask;

    // Read the interrupt status.
    if (AM_HAL_STATUS_SUCCESS != am_hal_adc_interrupt_status(g_ADCHandle, &ui32IntMask, false))
    {
        am_util_stdio_printf("Error reading ADC interrupt status\n");
    }

    // Clear the ADC interrupt.
    if (AM_HAL_STATUS_SUCCESS != am_hal_adc_interrupt_clear(g_ADCHandle, ui32IntMask))
    {
        am_util_stdio_printf("Error clearing ADC interrupt status\n");
    }

    // If we got a DMA complete, set the flag.
    if (ui32IntMask & AM_HAL_ADC_INT_DCMP)
    {
        g_bADCDMAComplete = true;
    }

    // If we got a DMA error, set the flag.
    if (ui32IntMask & AM_HAL_ADC_INT_DERR)
    {
        g_bADCDMAError = true;
    }
}
//*****************************************************************************
// Configure the ADC.
//*****************************************************************************
void adc_config_dma(void)
{
    am_hal_adc_dma_config_t       ADCDMAConfig;

    // Configure the ADC to use DMA for the sample transfer.
    ADCDMAConfig.bDynamicPriority = true;
    ADCDMAConfig.ePriority = AM_HAL_ADC_PRIOR_SERVICE_IMMED;
    ADCDMAConfig.bDMAEnable = true;
    ADCDMAConfig.ui32SampleCount = ADC_SAMPLE_BUF_SIZE * 3;
    ADCDMAConfig.ui32TargetAddress = (uint32_t)g_ui32ADCSampleBuffer;
    if (AM_HAL_STATUS_SUCCESS != am_hal_adc_configure_dma(g_ADCHandle, &ADCDMAConfig))
    {
        am_util_stdio_printf("Error - configuring ADC DMA failed.\n");
    }

    // Reset the ADC DMA flags.
    g_bADCDMAComplete = false;
    g_bADCDMAError = false;
}
//*****************************************************************************
// Configure the ADC
//*****************************************************************************
void adc_config(void)
{
    am_hal_adc_config_t           ADCConfig;
    am_hal_adc_slot_config_t      ADCSlotConfig;

    // Initialize the ADC and get the handle.
    if ( AM_HAL_STATUS_SUCCESS != am_hal_adc_initialize(0, &g_ADCHandle) )
    {
        am_util_stdio_printf("Error - reservation of the ADC instance failed.\n");
    }

    // Power on the ADC.
    if (AM_HAL_STATUS_SUCCESS != am_hal_adc_power_control(g_ADCHandle,AM_HAL_SYSCTRL_WAKE,false) )
    {
        am_util_stdio_printf("Error - ADC power on failed.\n");
    }

    // Set up the ADC configuration parameters. These settings are reasonable for accurate measurements at a low sample rate.
    ADCConfig.eClock             = AM_HAL_ADC_CLKSEL_HFRC;
    ADCConfig.ePolarity          = AM_HAL_ADC_TRIGPOL_RISING;
    ADCConfig.eTrigger           = AM_HAL_ADC_TRIGSEL_SOFTWARE;
    ADCConfig.eReference         = AM_HAL_ADC_REFSEL_INT_1P5;
    ADCConfig.eClockMode         = AM_HAL_ADC_CLKMODE_LOW_LATENCY;
    ADCConfig.ePowerMode         = AM_HAL_ADC_LPMODE0;
    ADCConfig.eRepeat            = AM_HAL_ADC_REPEATING_SCAN;
    if (AM_HAL_STATUS_SUCCESS != am_hal_adc_configure(g_ADCHandle, &ADCConfig))
    {
        am_util_stdio_printf("Error - configuring ADC failed.\n");
    }

    // Set up an ADC slot
    ADCSlotConfig.eMeasToAvg      = AM_HAL_ADC_SLOT_AVG_128;
    ADCSlotConfig.ePrecisionMode  = AM_HAL_ADC_SLOT_14BIT;
    ADCSlotConfig.eChannel        = AM_HAL_ADC_SLOT_CHSEL_SE0;
    ADCSlotConfig.bWindowCompare  = false;
    ADCSlotConfig.bEnabled        = true;
    if (AM_HAL_STATUS_SUCCESS != am_hal_adc_configure_slot(g_ADCHandle, 0, &ADCSlotConfig))
    {
        am_util_stdio_printf("Error - configuring ADC Slot 0 failed.\n");
    }

    // Slot 1 for Pin 29
    ADCSlotConfig.eChannel = AM_HAL_ADC_SLOT_CHSEL_SE1;
    if (AM_HAL_STATUS_SUCCESS != am_hal_adc_configure_slot(g_ADCHandle, 1, &ADCSlotConfig))
    {
        am_util_stdio_printf("Error - configuring ADC Slot 1 failed.\n");
    }

    // Slot 2 for Pin 11
    ADCSlotConfig.eChannel = AM_HAL_ADC_SLOT_CHSEL_SE2;
    if (AM_HAL_STATUS_SUCCESS != am_hal_adc_configure_slot(g_ADCHandle, 2, &ADCSlotConfig))
    {
        am_util_stdio_printf("Error - configuring ADC Slot 2 failed.\n");
    }

    // Configure the ADC to use DMA for the sample transfer.
    adc_config_dma();

    // For this example, the samples will be coming in slowly. This means we can afford to wake up for every conversion.
    am_hal_adc_interrupt_enable(g_ADCHandle, AM_HAL_ADC_INT_DERR | AM_HAL_ADC_INT_DCMP );

    // Enable the ADC.
    if (AM_HAL_STATUS_SUCCESS != am_hal_adc_enable(g_ADCHandle))
    {
        am_util_stdio_printf("Error - enabling ADC failed.\n");
    }
}
//*****************************************************************************
// Set or clear an LED.
// The LEDs on the EVB are common anode (anodes tied high).
// They are turned on when the transistor is turned on (i.e. high).
// Therefore, they are turned on when the GPIO is set.
// Likewise, they are turned off when the GPIO is cleared.
// NOTE: Polarity was opposite with the old, and now deprecated, EVK.
//*****************************************************************************
static void LED_on(uint32_t ui32LED, bool bOn)
{
    uint32_t ui32Mask;

    if ( ui32LED <= 31 )
    {
        ui32Mask = 0x01 << (ui32LED - 0);

        if ( bOn )
        {
            GPIO->WTSA = ui32Mask;
        }
        else
        {
            GPIO->WTCA = ui32Mask;
        }
    }
    else if ( ui32LED <= 49 )
    {
        ui32Mask = 0x01 << (ui32LED - 32);

        if ( bOn )
        {
            GPIO->WTSB = ui32Mask;
        }
        else
        {
            GPIO->WTCB = ui32Mask;
        }
    }
    else
    {
        // ERROR.
        while (1);
    }
} // LED_on()
//*****************************************************************************
// Set a PADREG FNCSEL field.
//*****************************************************************************
static void padreg_funcsel_set(uint32_t ui32GPIOnum, uint32_t ui32Func)
{
    uint32_t ui32Shift;
    uint32_t volatile *pui32Reg;

    // Unlock writes to the GPIO and PAD configuration registers.
    GPIO->PADKEY = 0x73;

    // Determine and configure the PADREG.
    // Each PADREG configures 4 GPIOs, each field divided into bytes.
    ui32Shift = (ui32GPIOnum & 0x3) * 8;

    // Since there are 4 GPIOs per PADREG, the GPIO number also happens to be a byte offset to the PADREG address. Since we have a word ptr, we need to make it a word offset (so divide by 4).
    pui32Reg   = &(GPIO->PADREGA) + (ui32GPIOnum / 4);

    // Set the padreg value given by the caller.
    *pui32Reg &= ~(0xFF << ui32Shift);
    *pui32Reg |= ui32Func << ui32Shift;

    // Lock PAD configuration registers.
    GPIO->PADKEY = 0;
}

//*****************************************************************************
// Write a GPIO CFG field.
//*****************************************************************************
static void gpio_cfg_set(uint32_t ui32GPIOnum, uint32_t ui32CFG)
{
    uint32_t ui32Shift;
    uint32_t volatile *pui32Reg;

    // Unlock writes to the GPIO and PAD configuration registers.
    GPIO->PADKEY = 0x73;

    // Configure the GPIO as push pull outputs for use with an LED. Each GPIOCFG configures 8 GPIOs, each divided into nibbles.

    ui32Shift  = (ui32GPIOnum & 0x7) * 4;
    pui32Reg   = &(GPIO->CFGA) + (ui32GPIOnum / 8);
    *pui32Reg &= ~(0xF << ui32Shift);

    // The OUTCFG field lsb is 1 bit into the CFG field. OUTCFG is 2 bits wide.
    *pui32Reg |= ((ui32CFG & 0x3) << ui32Shift);

    // Lock PAD configuration registers.
    GPIO->PADKEY = 0;
} // gpio_cfg_set()

//*****************************************************************************
// Configure a GPIO for use with an LED.
//*****************************************************************************
void LED_gpio_cfg(uint32_t ui32GPIOnum)
{
    if ( ui32GPIOnum > 49 )
    {
        // Error
        while(1);
    }

    // Configure the PAD for GPIO (FUNCSEL=3).
    // The 8-bit PADREG bitfields are generally:
    //  [7:6] = Pullup resistor selection.
    //  [5:3] = Function select (3 = GPIO).
    //  [2]   = Drive strength
    //  [1]   = INPEN
    //  [0]   = Pullup enable
    padreg_funcsel_set(ui32GPIOnum, 0x18);

    // Configure the GPIO output for PUSHPULL (OUTCFG=1).
    // CFG is 4 bits:
    // [3]   = INTD, 0=INTLH, 1=INTHL
    // [2:1] = OUTCFG. 0=DIS, 1=PUSHPULL, 2=open drain, 3=tristate
    // [0]   = READ
    gpio_cfg_set(ui32GPIOnum, 0x2);
} // LED_gpio_cfg()

//*****************************************************************************
// Initialize GPIO PADS to drive the LEDs and enable SWO
//*****************************************************************************
void GPIO_init(void)
{
    // Configure pads for LEDs as GPIO functions.
    LED_gpio_cfg(EVB_LED0);
    LED_gpio_cfg(EVB_LED1);
    LED_gpio_cfg(EVB_LED2);
    LED_gpio_cfg(EVB_LED3);
#ifdef EVB_LED4
    LED_gpio_cfg(EVB_LED4);
#endif // EVB_LED4

    // While we're at it, also configure GPIO41 for SWO.
    padreg_funcsel_set(41, 2);

    // Initialize the LEDs to all off.
    LED_on(EVB_LED0, false);
    LED_on(EVB_LED1, false);
    LED_on(EVB_LED2, false);
    LED_on(EVB_LED3, false);
#ifdef EVB_LED4
    LED_on(EVB_LED4, false);
#endif // EVB_LED4
} // GPIO_init()

//*****************************************************************************
// setSleepMode
//*****************************************************************************
void setSleepMode (int bSetDeepSleep)
{
  if (bSetDeepSleep)
  {
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
  }
  else
  {
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
  }

} // setSleepMode()
//*****************************************************************************
// Initialize the ADC repetitive sample timer A3.
//*****************************************************************************
void init_timerA3_for_ADC(void)
{
    // Start a timer to trigger the ADC periodically (1 second).
    am_hal_ctimer_config_single(3, AM_HAL_CTIMER_TIMERA, AM_HAL_CTIMER_HFRC_12MHZ| AM_HAL_CTIMER_FN_REPEAT | AM_HAL_CTIMER_INT_ENABLE);
    am_hal_ctimer_int_enable(AM_HAL_CTIMER_INT_TIMERA3);
    am_hal_ctimer_period_set(3, AM_HAL_CTIMER_TIMERA, 10, 5);

    // Enable the timer A3 to trigger the ADC directly
    am_hal_ctimer_adc_trigger_enable();

    // Start the timer.
    am_hal_ctimer_start(3, AM_HAL_CTIMER_TIMERA);
}
void delay_1sec() {
    // Assuming a 7 MHz CPU clock and 100 cycles per loop iteration
    volatile uint32_t count = 30*7000000; // 7/24/48/96 MHz * 1 sec
    while (count--);
}

float max_arr2(float *arr, int len) {
	float max_val = arr[0];
	for (int i = 1; i < len; i++) {
		if (arr[i] > max_val) {
			max_val = arr[i];
		}
	}
	return max_val;
}

float min_arr2(float *arr, int len) {
	float min_val = arr[0];
	for (int i = 0; i < len; i++) {
		if (arr[i] < min_val) {
			min_val = arr[i];
		}
	}
	return min_val;
}

float max_arr(uint32_t *arr, int len) {
	float max_val = arr[0];
	for (int i = 0; i < len; i++) {
		if (arr[i] > max_val) {
			max_val = arr[i];
		}
	}
	return max_val;
}

float min_arr(uint32_t *arr, int len) {
	float min_val = arr[0];
	for (int i = 1; i < len; i++) {
		if (arr[i] < min_val) {
			min_val = arr[i];
		}
	}
	return min_val;
}

float std_dev(uint32_t *arr, int n) {
	float sum = 0.0, mean, std_dev = 0.0;

	// Calculate the mean
	for (int i = 0; i < n; i++) {
		sum += arr[i];
	}
	mean = sum / n;

	// Calculate the standard deviation
	for (int i = 0; i < n; i++) {
		std_dev += pow(arr[i] - mean, 2);
	}
	std_dev = sqrt(std_dev / (n - 1));

	return std_dev;
}

void normalize_arr(uint32_t *arr, int n, float min_val, float max_val, float *arr_norm) {
	float range = max_val - min_val;
	for (int i = 0; i < n; i++) {
		arr_norm[i] = (arr[i] - min_val) / range;
	}
}

void get_nrfeat_window(uint32_t *wnd_ts, uint32_t *wnd_adc1, uint8_t *wnd_pir, uint8_t *wnd_drv, float *vmax1, float *vmin1, float *vmean1, float *vstd1, float *hpu1, float *hpd1) {
	short x_tam = PATCH_DUR;

	*vmax1 = max_arr(wnd_adc1, PATCH_DUR);
	*vmin1 = min_arr(wnd_adc1, PATCH_DUR);
	*vmean1 = *vmax1 - *vmin1;// mean_arr(yadc1, xx_tam);
	*vstd1 = std_dev(wnd_adc1, PATCH_DUR);
	*hpu1 = 0;
	*hpd1 = 0;

	for (short i = 0; i < x_tam - 1; i++) {
		float casted=wnd_adc1[i];
		if (casted >= *vmax1){
			*hpu1 = i;
		}
		if (casted <= *vmin1){
			*hpd1 = i;
		}
	}
}
	
void observation_processor_calib(uint32_t *ts_, uint8_t *drv_, uint32_t *adc_, uint8_t *pir_, uint32_t *obs_rt, bool *obs_pr, bool label) {
	*obs_pr = 0;

	//for (short i = 0; i < PATCH_DUR; i++) {
	//	am_util_stdio_printf("wnd_ts[i] %u, wnd_drv[i] %u,wnd_adc1[i] %u, wnd_pir[i] %u\n", ts_[i], drv_[i], adc_[i], pir_[i]);
	//}
	float vmax1, vmin1, vmean1, vstd1, hpu1, hpd1;
	get_nrfeat_window(ts_, adc_, pir_, drv_, &vmax1, &vmin1, &vmean1, &vstd1, &hpu1, &hpd1);
  am_util_stdio_printf("%f,%f,%f,%f,%f,%f\n", vmax1, vmin1, vmean1, vstd1, hpu1, hpd1);
	/*
	char filename[100];
	sprintf(filename, "%s_%s.txt", SAVE_FILE, (label == 1) ? "occ" : "uocc");
	printf("saving feature in file: %s\n", filename);
	FILE *file = fopen(filename, "a");
	if (file != NULL) {
		fprintf(file, "%ld,%f,%f,%f,%f,%f,%f\n", wnd_ts[*buf_ts_size - ST - 1], vmax1, vmin1, vmean1, vstd1, hpu1, hpd1);
		fclose(file);
	}
	else
	{
		printf("saving feature file failed at %s", filename);
	}*/
	
}
int obs_loop_calib()
{
	// Set the clock frequency.
	if (AM_HAL_STATUS_SUCCESS != am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_SYSCLK_MAX, 0))
	{
		am_util_stdio_printf("Error - configuring the system clock failed.\n");
	}

	// Set the default cache configuration and enable it.
	if (AM_HAL_STATUS_SUCCESS != am_hal_cachectrl_config(&am_hal_cachectrl_defaults))
	{
		am_util_stdio_printf("Error - configuring the system cache failed.\n");
	}

	if (AM_HAL_STATUS_SUCCESS != am_hal_cachectrl_enable())
	{
		am_util_stdio_printf("Error - enabling the system cache failed.\n");
	}

  // Start the ITM interface.
  am_bsp_itm_printf_enable();

  // Start the CTIMER A3 for timer-based ADC measurements.
  init_timerA3_for_ADC();
	stimer_init(); 

  // Enable interrupts.
  NVIC_EnableIRQ(ADC_IRQn);
  am_hal_interrupt_master_enable();

  // Set a pin to act as our ADC input
	am_hal_gpio_pinconfig(16, g_AM_PIN_16_ADCSE0);
	am_hal_gpio_pinconfig(29, g_AM_PIN_29_ADCSE1);
	am_hal_gpio_pinconfig(11, g_AM_PIN_11_ADCSE2);

  // Configure the ADC
  adc_config();

  // Trigger the ADC sampling for the first time manually.
	if (AM_HAL_STATUS_SUCCESS != am_hal_adc_sw_trigger(g_ADCHandle))
	{
			am_util_stdio_printf("Error - triggering the ADC failed.\n");
	}

  // Print the banner.
  //am_util_stdio_terminal_clear();	
	
  am_util_stdio_printf("All initialized.\n");

	short turn = 0;

	uint32_t ts_st=0, ts_end=0;
	while (turn <= 1) 
	{
		if (turn > 0) 
		{
			if (TARGET_CLASS == 0) 
			{
				//printf("Unoccupancy calibrated.Moving to occupancy calibration.\n");
				LED_on(EVB_LED0, true); 
			}
			else 
			{
				//printf("Occupancy calibrated.Moving to unoccupancy calibration.\n");
				LED_on(EVB_LED0, true); 
			}
		}
		turn = turn + 1;
		uint32_t drv[ADC_SAMPLE_BUF_SIZE] = { 0 }, adc1_r[ADC_SAMPLE_BUF_SIZE] = { 0 }, pir[ADC_SAMPLE_BUF_SIZE] = { 0 }, ts[ADC_SAMPLE_BUF_SIZE] = { 0 };
		uint32_t adc_[PATCH_DUR] = { 0 }, ts_[PATCH_DUR] = { 0 };
		uint8_t drv_[PATCH_DUR] = { 0 }, pir_[PATCH_DUR] = { 0 };
    uint32_t  avg_drv = 0, avg_adc = 0, avg_pir=0;
		short pair_size = 0, drv_size = 0, pir_size = 0, adc1_r_size = 0, ts_size = 0;
		bool detectR = false; //false="Unoccupied(PIR)", true="Occupied(PIR)"
		bool process_sample = false;
		uint32_t timestamp_l=0;
		uint32_t patch_ctr=0;
		uint32_t tm_diff=0;
		while (1) 
		{//super loop
			bool slpr_op = false;
			if (process_sample == true) 
			{
				avg_drv=0;
				avg_adc=0;
				tm_diff=0;
				for (short i = 0; i < ADC_SAMPLE_BUF_SIZE; i++) 
				{
					avg_adc=avg_adc+adc1_r[i];
					if (drv[i] > 1000)
						avg_drv = avg_drv + 1;
					else
						avg_drv = avg_drv + 0;
					if (pir[i] > 1000)
						avg_pir = avg_pir + 1;
					else
						avg_pir = avg_pir + 0;
					//am_util_stdio_printf("pir:%u, avg_pir:%u\n",pir[i],avg_pir);
				}
				ts_size = ADC_SAMPLE_BUF_SIZE;
				adc1_r_size = ADC_SAMPLE_BUF_SIZE;
				pir_size = ADC_SAMPLE_BUF_SIZE;
				drv_size = ADC_SAMPLE_BUF_SIZE;

				if ((avg_drv/drv_size) > 0.5)
					avg_drv=1;
				else
					avg_drv=0;
				avg_adc=avg_adc/adc1_r_size;
				float val=((float)avg_pir/(float)pir_size);
				if (val > 0.1)
					avg_pir=1;
				else
					avg_pir=0;
				//am_util_stdio_printf("val:%f, (avg_pir/pir_size):%f, avg_pir:%u\n",val, (avg_pir/pir_size),avg_pir);

				/*for (short i = 0; i < adc1_r_size; i++) 
				{
					adc1_r[i] = adc1_r[i] / 50.0;
				}*/
				// Detection algorithm

				if ((patch_ctr > 0) && (avg_drv == 0))
				{
					patch_ctr=0;
				}
				//am_util_stdio_printf("patch_ctr: %u, %u,%u,%u,%u\n", patch_ctr, ts[0], avg_drv, avg_adc, avg_pir);			
				if (avg_drv > 0)
				{
					if (patch_ctr > 0)
						tm_diff=ts[patch_ctr-1]-ts[0];	
					ts_[patch_ctr]=ts[0];
					drv_[patch_ctr]=avg_drv;
					adc_[patch_ctr]=avg_adc;
					pir_[patch_ctr]=avg_pir;
					patch_ctr++;
					//am_util_stdio_printf("patch_ctr: %u, %u,%u,%u,%u\n", ts[0], patch_ctr, avg_drv, avg_adc, avg_pir);			

				}
				
				if (patch_ctr >= PATCH_DUR)
				{
					am_util_stdio_printf("1\n");

					patch_ctr=0;
					//consolidate(ts, &ts_size, adc1_r, &adc1_r_size, pir, &pir_size, drv, &drv_size);				
					
					short act_pir = 0;
					for (short i = 0; i < PATCH_DUR; i++) 
					{
						//am_util_stdio_printf("%u\n",pir_[i]);
						if (adc_[i]>15000)
							act_pir = act_pir + 1;
					}

					float val=(float)PIR_SENST*(float)PATCH_DUR;
					am_util_stdio_printf("%f,%u\n", val, act_pir);			

					if (act_pir > val) 
					{
						detectR = true;
					}else{
						detectR = false;
					}

					/*for (short i = 0; i < PATCH_DUR; i++) {
						am_util_stdio_printf("ts[i] %u, drv[i] %u,adc1_r[i] %u, pir[i] %u\n", ts_[i], drv_[i], adc_[i], pir_[i]);
					}*/

					if (detectR == false) 
					{
						//am_util_stdio_printf("No motion detected\n");
						bool obs_pr = 0;
						uint32_t obs_rt[MAX_FEATURES] = { 0 };
						observation_processor_calib(ts_, drv_, adc_, pir_, obs_rt, &obs_pr, TARGET_CLASS);
					}
				}
				if (detectR == true) 
				{
					LED_on(EVB_LED4, true); 
					am_util_stdio_printf("Motion Detected\n");
				}
				else 
				{
					LED_on(EVB_LED4, false); 
					//printf("MOTION-LOW\n");
				}

				//Reset variables
				slpr_op = false;
				detectR = false;
				process_sample = false;
			}
			// ADC DMA implementation starts here
			// Feel free to implement any functions that will be called within the following implementation 

			// Check for DMA errors.
			if (g_bADCDMAError)
			{
				am_util_stdio_printf("DMA Error occured\n");
				while(1);
			}
				
			// Check if the ADC DMA completion interrupt occurred. Use the logic implemented in the adc_example.c
			if (g_bADCDMAComplete)
			{
				uint32_t ui32SampleCount;
				// Get the relative current timestamp using STIMER documentation
				uint32_t timestamp = am_hal_stimer_counter_get(); // Fetch current timer value
				if (timestamp_l == 0)
						timestamp_l=timestamp;
				//am_util_stdio_printf("DMA Complete\n");
				ui32SampleCount = ADC_SAMPLE_BUF_SIZE*3;
				if (AM_HAL_STATUS_SUCCESS == am_hal_adc_samples_read(g_ADCHandle, false, g_ui32ADCSampleBuffer, &ui32SampleCount, SampleBuffer))
				{
						short j=0;
						uint32_t tim=0;
						for (short i = 0; i < ui32SampleCount; i += 3)
						{
							ts[j]=timestamp+tim;
							j++;
							if (((timestamp-timestamp_l) > 4000) || ((timestamp-timestamp_l) < -40000))
							{
									tim++;
									timestamp_l=timestamp;
							}
							
							//am_util_stdio_printf("Sample %u: %u\n", i / 3, SampleBuffer[i].ui32Sample);
						}
   					// Assuming the samples are stored sequentially for each channel.			
						//am_util_stdio_printf("Channel SE0 Data (Pin 16):\n");
					  j=0;
						for (short i = 0; i < ui32SampleCount; i += 3)
						{
							adc1_r[j]=SampleBuffer[i].ui32Sample;
							j++;
							//am_util_stdio_printf("Sample %u: %u\n", i / 3, SampleBuffer[i].ui32Sample);
						}
						//am_util_stdio_printf("Channel SE1 Data (Pin 29):\n");
						j=0;
						for (short i = 1; i < ui32SampleCount; i += 3)
						{
							pir[j]=SampleBuffer[i].ui32Sample;
							j++;
							//am_util_stdio_printf("Sample %u: %u\n", i / 3, SampleBuffer[i].ui32Sample);
						}				
						//am_util_stdio_printf("Channel SE2 Data (Pin 11):\n");                    
						j=0;
						for (short i = 2; i < ui32SampleCount; i += 3)
						{
							drv[j]=SampleBuffer[i].ui32Sample;
							j++;							
							//am_util_stdio_printf("Sample %u: %u\n", i / 3, SampleBuffer[i].ui32Sample);
						}
						/*am_util_stdio_printf("Timestamp: %u\n", timestamp);
						for (uint32_t i = 1; i < ui32SampleCount; i +=3)
						{
								if (SampleBuffer[i].ui32Sample > 2000)
									am_util_stdio_printf("Sample %u: %u\n", i, SampleBuffer[i].ui32Sample);
						}*/
				}
				else 
				{
						am_util_stdio_printf("Error - failed to process samples.\n");
				}
				
				// Reset the DMA completion and error flags.
				g_bADCDMAComplete = false;

				// Re-configure the ADC DMA.
				adc_config_dma();

				// Clear the ADC interrupts.
				if (AM_HAL_STATUS_SUCCESS != am_hal_adc_interrupt_clear(g_ADCHandle, 0xFFFFFFFF))
				{
						am_util_stdio_printf("Error - clearing the ADC interrupts failed.\n");
				}

				// Trigger the ADC sampling for the first time manually.
				if (AM_HAL_STATUS_SUCCESS != am_hal_adc_sw_trigger(g_ADCHandle))
				{
						am_util_stdio_printf("Error - triggering the ADC failed.\n");
				}			
				process_sample = true;
			}
			// ADC DMA reading implementation ends here
		}
	}
	return 0;
}
//*****************************************************************************
//
// Main
//
//*****************************************************************************
int main(void)
{
    // Initialize GPIO PADs we will use.
    GPIO_init();
		//delay_1sec();		
		//LED_on(EVB_LED4, true);
    
		int error = 0;
		if (TARGET_CLASS == -1)
			//error = obs_loop();
			error=0;
		else
			error = obs_loop_calib();
		return 0;
	  
    // Configure type of sleep (0=normal sleep, 1=deep sleep).
    //setSleepMode(1);
}
