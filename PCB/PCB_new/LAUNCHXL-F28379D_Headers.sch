EESchema Schematic File Version 4
LIBS:PCB_2020-cache
EELAYER 30 0
EELAYER END
$Descr User 12505 8433
encoding utf-8
Sheet 2 3
Title "AMP Headers"
Date "2019-12-13"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text GLabel 8750 3500 2    39   BiDi ~ 0
SCIB_RX
$Comp
L Mechanical:MountingHole H1
U 1 1 5E1FD379
P 1150 2200
F 0 "H1" H 1250 2246 50  0000 L CNN
F 1 "MountingHole" H 1250 2155 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.7mm" H 1150 2200 50  0001 C CNN
F 3 "https://www.adafruit.com/product/3299" H 1150 2200 50  0001 C CNN
	1    1150 2200
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H2
U 1 1 5E27C5D4
P 1150 2500
F 0 "H2" H 1250 2546 50  0000 L CNN
F 1 "MountingHole" H 1250 2455 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.7mm" H 1150 2500 50  0001 C CNN
F 3 "https://www.adafruit.com/product/3299" H 1150 2500 50  0001 C CNN
	1    1150 2500
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H3
U 1 1 5E27C92D
P 1150 2800
F 0 "H3" H 1250 2846 50  0000 L CNN
F 1 "MountingHole" H 1250 2755 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.7mm" H 1150 2800 50  0001 C CNN
F 3 "https://www.adafruit.com/product/3299" H 1150 2800 50  0001 C CNN
	1    1150 2800
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H4
U 1 1 5E27CB44
P 1150 3100
F 0 "H4" H 1250 3146 50  0000 L CNN
F 1 "MountingHole" H 1250 3055 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.7mm" H 1150 3100 50  0001 C CNN
F 3 "https://www.adafruit.com/product/3299" H 1150 3100 50  0001 C CNN
	1    1150 3100
	1    0    0    -1  
$EndComp
Text GLabel 8750 3400 2    39   BiDi ~ 0
SCIB_TX
Text GLabel 8750 3100 2    39   Output ~ 0
Brake_PWM_1
Text GLabel 8750 3200 2    39   Output ~ 0
Brake_PWM_2
Wire Wire Line
	8150 4100 8150 3950
Wire Wire Line
	6950 4100 6950 3950
Wire Wire Line
	8450 4100 8450 3950
Text GLabel 8750 3300 2    39   BiDi ~ 0
STEERING
Text GLabel 8150 4100 3    39   Output ~ 0
EN+_CTL
Text GLabel 7850 4100 3    39   Input ~ 0
INPUT_A+_CTL
Text GLabel 7550 4100 3    39   BiDi ~ 0
FS1_CTL
Text GLabel 7250 4100 3    39   BiDi ~ 0
FWD_CTL
Text GLabel 8450 4100 3    39   BiDi ~ 0
REV_CTL
Wire Wire Line
	7850 3950 7850 4100
Wire Wire Line
	7550 3950 7550 4100
Wire Wire Line
	7250 3950 7250 4100
Text GLabel 6450 3300 0    39   Output ~ 0
THROTTLE
Wire Wire Line
	6450 3300 6650 3300
Wire Wire Line
	8650 3100 8750 3100
Wire Wire Line
	8650 3200 8750 3200
Wire Wire Line
	8650 3300 8750 3300
$Comp
L power:GND #GND?
U 1 1 5E681F81
P 7700 2750
AR Path="/5E681F81" Ref="#GND?"  Part="1" 
AR Path="/5DF27589/5E681F81" Ref="#GND04"  Part="1" 
F 0 "#GND04" H 7700 2750 50  0001 C CNN
F 1 "GND" H 7600 2550 59  0000 L BNN
F 2 "" H 7700 2750 50  0001 C CNN
F 3 "" H 7700 2750 50  0001 C CNN
	1    7700 2750
	-1   0    0    1   
$EndComp
$Comp
L power:+5V #P+?
U 1 1 5E684B6F
P 7500 2850
AR Path="/5E684B6F" Ref="#P+?"  Part="1" 
AR Path="/5DF27589/5E684B6F" Ref="#P+06"  Part="1" 
F 0 "#P+06" H 7500 2850 50  0001 C CNN
F 1 "+5V" H 7400 3000 59  0000 L BNN
F 2 "" H 7500 2850 50  0001 C CNN
F 3 "" H 7500 2850 50  0001 C CNN
	1    7500 2850
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR01
U 1 1 5E686230
P 7950 2850
F 0 "#PWR01" H 7950 2700 50  0001 C CNN
F 1 "+12V" H 7850 3050 50  0000 L CNN
F 2 "" H 7950 2850 50  0001 C CNN
F 3 "" H 7950 2850 50  0001 C CNN
	1    7950 2850
	1    0    0    -1  
$EndComp
Text GLabel 6950 4100 3    39   BiDi ~ 0
KS1_CTL
NoConn ~ 6650 3400
Wire Wire Line
	7650 2850 7650 2750
Wire Wire Line
	7650 2750 7700 2750
Wire Wire Line
	7800 2750 7800 2850
Connection ~ 7700 2750
Wire Wire Line
	7700 2750 7800 2750
Wire Wire Line
	8650 3400 8750 3400
Wire Wire Line
	8650 3500 8750 3500
NoConn ~ 7350 2850
$Comp
L PCB_2020:AMP_DUE_SCH U1
U 6 1 5E7098FE
P 7650 3400
F 0 "U1" H 6606 3446 50  0000 R CNN
F 1 "AMP_DUE_SCH" H 6606 3355 50  0000 R CNN
F 2 "Drivetrain_control:Arduino DUE" H 7650 3400 50  0001 C CNN
F 3 "" H 8200 3300 50  0001 C CNN
	6    7650 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	3300 5700 3300 5950
Text Notes 3650 7200 0    59   ~ 0
Decoupling Caps
$Comp
L power:+3V3 #+3V2
U 1 1 684850F5
P 3300 5700
AR Path="/684850F5" Ref="#+3V2"  Part="1" 
AR Path="/5DF27589/684850F5" Ref="#+3V02"  Part="1" 
F 0 "#+3V02" H 3300 5700 50  0001 C CNN
F 1 "+3V3" H 3150 5850 59  0000 L BNN
F 2 "" H 3300 5700 50  0001 C CNN
F 3 "" H 3300 5700 50  0001 C CNN
	1    3300 5700
	1    0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 6A2D04C9
P 3300 6100
AR Path="/6A2D04C9" Ref="C2"  Part="1" 
AR Path="/5DF27589/6A2D04C9" Ref="C2"  Part="1" 
F 0 "C2" H 3360 6115 59  0000 L BNN
F 1 "0.1uF" H 3360 5915 59  0000 L BNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 3300 6100 50  0001 C CNN
F 3 "https://www.digikey.com/product-detail/en/samsung-electro-mechanics/CL21B104JBCNNNC/1276-1090-1-ND/3889176?utm_adgroup=Capacitors&slid=&pdv=c&pcrid=398891859082&productid=&utm_campaign=Dynamic+Search&mkwid=sK4nOncLn&utm_medium=cpc&utm_term=&gclid=CjwKCAiAmNbwBRBOEiwAqcwwpZSQgtNujl6BHsUYP-FQ70zdNcKZbQcVdbZiRuST7tQ2_ezynANN3xoCFdYQAvD_BwE&pmt=b&pkw=&utm_source=google" H 3300 6100 50  0001 C CNN
	1    3300 6100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #GND3
U 1 1 E2B99DEF
P 4050 6500
AR Path="/E2B99DEF" Ref="#GND3"  Part="1" 
AR Path="/5DF27589/E2B99DEF" Ref="#GND03"  Part="1" 
F 0 "#GND03" H 4050 6500 50  0001 C CNN
F 1 "GND" H 3950 6300 59  0000 L BNN
F 2 "" H 4050 6500 50  0001 C CNN
F 3 "" H 4050 6500 50  0001 C CNN
	1    4050 6500
	1    0    0    -1  
$EndComp
Wire Wire Line
	3300 6500 4050 6500
$Comp
L Device:C C5
U 1 1 6C4A4E38
P 4750 6100
AR Path="/6C4A4E38" Ref="C5"  Part="1" 
AR Path="/5DF27589/6C4A4E38" Ref="C5"  Part="1" 
F 0 "C5" H 4810 6115 59  0000 L BNN
F 1 "0.1uF" H 4810 5915 59  0000 L BNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 4750 6100 50  0001 C CNN
F 3 "https://www.digikey.com/product-detail/en/samsung-electro-mechanics/CL21B104JBCNNNC/1276-1090-1-ND/3889176?utm_adgroup=Capacitors&slid=&pdv=c&pcrid=398891859082&productid=&utm_campaign=Dynamic+Search&mkwid=sK4nOncLn&utm_medium=cpc&utm_term=&gclid=CjwKCAiAmNbwBRBOEiwAqcwwpZSQgtNujl6BHsUYP-FQ70zdNcKZbQcVdbZiRuST7tQ2_ezynANN3xoCFdYQAvD_BwE&pmt=b&pkw=&utm_source=google" H 4750 6100 50  0001 C CNN
	1    4750 6100
	1    0    0    -1  
$EndComp
Wire Wire Line
	4750 6250 4750 6500
$Comp
L power:+12V #PWR0101
U 1 1 5E5B7DE1
P 4750 5700
F 0 "#PWR0101" H 4750 5550 50  0001 C CNN
F 1 "+12V" H 4650 5900 50  0000 L CNN
F 2 "" H 4750 5700 50  0001 C CNN
F 3 "" H 4750 5700 50  0001 C CNN
	1    4750 5700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4750 5700 4750 5950
Wire Wire Line
	3300 6250 3300 6500
Connection ~ 4050 6500
Wire Wire Line
	4050 6500 4100 6500
Wire Wire Line
	4050 6500 4750 6500
$EndSCHEMATC
