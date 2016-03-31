/*
 * Copyright (c) 2011-2012, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/mfd/pm8xxx/pm8xxx-adc.h>
#ifdef CONFIG_BATTERY_SH
#include "sharp/sh_smem.h"
#endif /* CONFIG_BATTERY_SH */

#define KELVINMIL_DEGMIL	273160

/* Units for temperature below (on x axis) is in 0.1DegC as
   required by the battery driver. Note the resolution used
   here to compute the table was done for DegC to milli-volts.
   In consideration to limit the size of the table for the given
   temperature range below, the result is linearly interpolated
   and provided to the battery driver in the units desired for
   their framework which is 0.1DegC. True resolution of 0.1DegC
   will result in the below table size to increase by 10 times */
#ifdef CONFIG_BATTERY_SH
static const struct pm8xxx_adc_map_pt adcmap_batttherm[] = {
	{1800,	-55},
	{1659,	-20},
	{1593,	-10},
	{1509,	  0},
	{1400,	 10},
	{1271,	 20},
	{1125,	 30},
	{972,	 40},
	{820,	 50},
	{680,	 60},
	{553,	 70},
	{0,	122}
};

static const struct pm8xxx_adc_map_pt adcmap_batttherm_custom_1[] = {
	{1800,	-41},
	{1674,	-20},
	{1582,	-10},
	{1444,	  0},
	{1264,	 10},
	{1059,	 20},
	{848,	 30},
	{654,	 40},
	{491,	 50},
	{357,	 60},
	{257,	 70},
	{0,	84}
};

static const struct pm8xxx_adc_map_pt adcmap_batttherm_custom_2[] = {
	{1800,	-40},
	{1615,	-20},
	{1514,	-10},
	{1389,	  0},
	{1244,	 10},
	{1081,	 20},
	{917,	 30},
	{759,	 40},
	{616,	 50},
	{495,	 60},
	{397,	 70},
	{0,	122}
};

static const struct pm8xxx_adc_map_pt adcmap_batttherm_custom_3[] = {
	{1800,	-41},
	{1690,	-20},
	{1627,	-10},
	{1539,	  0},
	{1434,	 10},
	{1304,	 20},
	{1161,	 30},
	{1008,	 40},
	{854,	 50},
	{714,	 60},
	{591,	 70},
	{0,	122}
};
#endif /* CONFIG_BATTERY_SH */

static const struct pm8xxx_adc_map_pt adcmap_btm_threshold[] = {
	{-300,	1642},
	{-200,	1544},
	{-100,	1414},
	{0,	1260},
	{10,	1244},
	{20,	1228},
	{30,	1212},
	{40,	1195},
	{50,	1179},
	{60,	1162},
	{70,	1146},
	{80,	1129},
	{90,	1113},
	{100,	1097},
	{110,	1080},
	{120,	1064},
	{130,	1048},
	{140,	1032},
	{150,	1016},
	{160,	1000},
	{170,	985},
	{180,	969},
	{190,	954},
	{200,	939},
	{210,	924},
	{220,	909},
	{230,	894},
	{240,	880},
	{250,	866},
	{260,	852},
	{270,	838},
	{280,	824},
	{290,	811},
	{300,	798},
	{310,	785},
	{320,	773},
	{330,	760},
	{340,	748},
	{350,	736},
	{360,	725},
	{370,	713},
	{380,	702},
	{390,	691},
	{400,	681},
	{410,	670},
	{420,	660},
	{430,	650},
	{440,	640},
	{450,	631},
	{460,	622},
	{470,	613},
	{480,	604},
	{490,	595},
	{500,	587},
	{510,	579},
	{520,	571},
	{530,	563},
	{540,	556},
	{550,	548},
	{560,	541},
	{570,	534},
	{580,	527},
	{590,	521},
	{600,	514},
	{610,	508},
	{620,	502},
	{630,	496},
	{640,	490},
	{650,	485},
	{660,	281},
	{670,	274},
	{680,	267},
	{690,	260},
	{700,	254},
	{710,	247},
	{720,	241},
	{730,	235},
	{740,	229},
	{750,	224},
	{760,	218},
	{770,	213},
	{780,	208},
	{790,	203}
};

#ifdef CONFIG_BATTERY_SH
static const struct pm8xxx_adc_map_pt adcmap_camtemp[] = {
	{1800,	-45},
	{1686,	-20},
	{1614,	-10},
	{1507,	  0},
	{1351,	 10},
	{1167,	 20},
	{960,	 30},
	{763,	 40},
	{575,	 50},
	{428,	 60},
	{314,	 70},
	{0,	106}
};

#ifdef CONFIG_PM_MPP_12_USE_BACKLIGHT_THERM
static const struct pm8xxx_adc_map_pt adcmap_backlight_temp[] = {
	{1800,	-40},
	{1772,	-40},
	{1761,	-35},
	{1745,	-30},
	{1725,	-25},
	{1699,	-20},
	{1666,	-15},
	{1624,	-10},
	{1573,	-5},
	{1512,	0},
	{1440,	5},
	{1359,	10},
	{1269,	15},
	{1172,	20},
	{1071,	25},
	{968,	30},
	{866,	35},
	{768,	40},
	{676,	45},
	{590,	50},
	{512,	55},
	{443,	60},
	{382,	65},
	{328,	70},
	{282,	75},
	{242,	80},
	{207,	85},
	{178,	90},
	{153,	95},
	{132,	100},
	{113,	105},
	{98,	110},
	{85,	115},
	{74,	120},
	{64,	125},
	{0,	125}
};
#endif /* CONFIG_PM_MPP_12_USE_BACKLIGHT_THERM */

#endif /* CONFIG_BATTERY_SH */

#ifdef CONFIG_BATTERY_SH
static const struct pm8xxx_adc_map_pt adcmap_pa_therm[] = {
	{1800,	-46},
	{1593,	-20},
	{1486,	-10},
	{1346,	  0},
	{1181,	 10},
	{1002,	 20},
	{828,	 30},
	{662,	 40},
	{524,	 50},
	{408,	 60},
	{319,	 70},
	{0,	112}
};

static const struct pm8xxx_adc_map_pt adcmap_pa_therm_custom_1[] = {
	{1800,	-46},
	{1788,	-20},
	{1780,	-10},
	{1766,	  0},
	{1743,	 10},
	{1709,	 20},
	{1657,	 30},
	{1586,	 40},
	{1491,	 50},
	{1375,	 60},
	{1236,	 70},
	{0,	130}
};

static const struct pm8xxx_adc_map_pt adcmap_pa_therm_custom_2[] = {
	{1800,	-46},
	{1646,	-20},
	{1545,	-10},
	{1395,	  0},
	{1213,	 10},
	{998,	 20},
	{790,	 30},
	{590,	 40},
	{442,	 50},
	{312,	 60},
	{220,	 70},
	{0,	103}
};
#else  /* CONFIG_BATTERY_SH */
static const struct pm8xxx_adc_map_pt adcmap_pa_therm[] = {
	{1731,	-30},
	{1726,	-29},
	{1721,	-28},
	{1715,	-27},
	{1710,	-26},
	{1703,	-25},
	{1697,	-24},
	{1690,	-23},
	{1683,	-22},
	{1675,	-21},
	{1667,	-20},
	{1659,	-19},
	{1650,	-18},
	{1641,	-17},
	{1632,	-16},
	{1622,	-15},
	{1611,	-14},
	{1600,	-13},
	{1589,	-12},
	{1577,	-11},
	{1565,	-10},
	{1552,	-9},
	{1539,	-8},
	{1525,	-7},
	{1511,	-6},
	{1496,	-5},
	{1481,	-4},
	{1465,	-3},
	{1449,	-2},
	{1432,	-1},
	{1415,	0},
	{1398,	1},
	{1380,	2},
	{1362,	3},
	{1343,	4},
	{1324,	5},
	{1305,	6},
	{1285,	7},
	{1265,	8},
	{1245,	9},
	{1224,	10},
	{1203,	11},
	{1182,	12},
	{1161,	13},
	{1139,	14},
	{1118,	15},
	{1096,	16},
	{1074,	17},
	{1052,	18},
	{1030,	19},
	{1008,	20},
	{986,	21},
	{964,	22},
	{943,	23},
	{921,	24},
	{899,	25},
	{878,	26},
	{857,	27},
	{836,	28},
	{815,	29},
	{794,	30},
	{774,	31},
	{754,	32},
	{734,	33},
	{714,	34},
	{695,	35},
	{676,	36},
	{657,	37},
	{639,	38},
	{621,	39},
	{604,	40},
	{586,	41},
	{570,	42},
	{553,	43},
	{537,	44},
	{521,	45},
	{506,	46},
	{491,	47},
	{476,	48},
	{462,	49},
	{448,	50},
	{435,	51},
	{421,	52},
	{409,	53},
	{396,	54},
	{384,	55},
	{372,	56},
	{361,	57},
	{350,	58},
	{339,	59},
	{329,	60},
	{318,	61},
	{309,	62},
	{299,	63},
	{290,	64},
	{281,	65},
	{272,	66},
	{264,	67},
	{256,	68},
	{248,	69},
	{240,	70},
	{233,	71},
	{226,	72},
	{219,	73},
	{212,	74},
	{206,	75},
	{199,	76},
	{193,	77},
	{187,	78},
	{182,	79},
	{176,	80},
	{171,	81},
	{166,	82},
	{161,	83},
	{156,	84},
	{151,	85},
	{147,	86},
	{142,	87},
	{138,	88},
	{134,	89},
	{130,	90},
	{126,	91},
	{122,	92},
	{119,	93},
	{115,	94},
	{112,	95},
	{109,	96},
	{106,	97},
	{103,	98},
	{100,	99},
	{97,	100},
	{94,	101},
	{91,	102},
	{89,	103},
	{86,	104},
	{84,	105},
	{82,	106},
	{79,	107},
	{77,	108},
	{75,	109},
	{73,	110},
	{71,	111},
	{69,	112},
	{67,	113},
	{65,	114},
	{64,	115},
	{62,	116},
	{60,	117},
	{59,	118},
	{57,	119},
	{56,	120},
	{54,	121},
	{53,	122},
	{51,	123},
	{50,	124},
	{49,	125}
};
#endif /* CONFIG_BATTERY_SH */

static const struct pm8xxx_adc_map_pt adcmap_ntcg_104ef_104fb[] = {
	{696483,	-40960},
	{649148,	-39936},
	{605368,	-38912},
	{564809,	-37888},
	{527215,	-36864},
	{492322,	-35840},
	{460007,	-34816},
	{429982,	-33792},
	{402099,	-32768},
	{376192,	-31744},
	{352075,	-30720},
	{329714,	-29696},
	{308876,	-28672},
	{289480,	-27648},
	{271417,	-26624},
	{254574,	-25600},
	{238903,	-24576},
	{224276,	-23552},
	{210631,	-22528},
	{197896,	-21504},
	{186007,	-20480},
	{174899,	-19456},
	{164521,	-18432},
	{154818,	-17408},
	{145744,	-16384},
	{137265,	-15360},
	{129307,	-14336},
	{121866,	-13312},
	{114896,	-12288},
	{108365,	-11264},
	{102252,	-10240},
	{96499,		-9216},
	{91111,		-8192},
	{86055,		-7168},
	{81308,		-6144},
	{76857,		-5120},
	{72660,		-4096},
	{68722,		-3072},
	{65020,		-2048},
	{61538,		-1024},
	{58261,		0},
	{55177,		1024},
	{52274,		2048},
	{49538,		3072},
	{46962,		4096},
	{44531,		5120},
	{42243,		6144},
	{40083,		7168},
	{38045,		8192},
	{36122,		9216},
	{34308,		10240},
	{32592,		11264},
	{30972,		12288},
	{29442,		13312},
	{27995,		14336},
	{26624,		15360},
	{25333,		16384},
	{24109,		17408},
	{22951,		18432},
	{21854,		19456},
	{20807,		20480},
	{19831,		21504},
	{18899,		22528},
	{18016,		23552},
	{17178,		24576},
	{16384,		25600},
	{15631,		26624},
	{14916,		27648},
	{14237,		28672},
	{13593,		29696},
	{12976,		30720},
	{12400,		31744},
	{11848,		32768},
	{11324,		33792},
	{10825,		34816},
	{10354,		35840},
	{9900,		36864},
	{9471,		37888},
	{9062,		38912},
	{8674,		39936},
	{8306,		40960},
	{7951,		41984},
	{7616,		43008},
	{7296,		44032},
	{6991,		45056},
	{6701,		46080},
	{6424,		47104},
	{6160,		48128},
	{5908,		49152},
	{5667,		50176},
	{5439,		51200},
	{5219,		52224},
	{5010,		53248},
	{4810,		54272},
	{4619,		55296},
	{4440,		56320},
	{4263,		57344},
	{4097,		58368},
	{3938,		59392},
	{3785,		60416},
	{3637,		61440},
	{3501,		62464},
	{3368,		63488},
	{3240,		64512},
	{3118,		65536},
	{2998,		66560},
	{2889,		67584},
	{2782,		68608},
	{2680,		69632},
	{2581,		70656},
	{2490,		71680},
	{2397,		72704},
	{2310,		73728},
	{2227,		74752},
	{2147,		75776},
	{2064,		76800},
	{1998,		77824},
	{1927,		78848},
	{1860,		79872},
	{1795,		80896},
	{1736,		81920},
	{1673,		82944},
	{1615,		83968},
	{1560,		84992},
	{1507,		86016},
	{1456,		87040},
	{1407,		88064},
	{1360,		89088},
	{1314,		90112},
	{1271,		91136},
	{1228,		92160},
	{1189,		93184},
	{1150,		94208},
	{1112,		95232},
	{1076,		96256},
	{1042,		97280},
	{1008,		98304},
	{976,		99328},
	{945,		100352},
	{915,		101376},
	{886,		102400},
	{859,		103424},
	{832,		104448},
	{807,		105472},
	{782,		106496},
	{756,		107520},
	{735,		108544},
	{712,		109568},
	{691,		110592},
	{670,		111616},
	{650,		112640},
	{631,		113664},
	{612,		114688},
	{594,		115712},
	{577,		116736},
	{560,		117760},
	{544,		118784},
	{528,		119808},
	{513,		120832},
	{498,		121856},
	{483,		122880},
	{470,		123904},
	{457,		124928},
	{444,		125952},
	{431,		126976},
	{419,		128000}
};

static int32_t pm8xxx_adc_map_linear(const struct pm8xxx_adc_map_pt *pts,
		uint32_t tablesize, int32_t input, int64_t *output)
{
	bool descending = 1;
	uint32_t i = 0;

	if ((pts == NULL) || (output == NULL))
		return -EINVAL;

	/* Check if table is descending or ascending */
	if (tablesize > 1) {
		if (pts[0].x < pts[1].x)
			descending = 0;
	}

	while (i < tablesize) {
		if ((descending == 1) && (pts[i].x < input)) {
			/* table entry is less than measured
				value and table is descending, stop */
			break;
		} else if ((descending == 0) &&
				(pts[i].x > input)) {
			/* table entry is greater than measured
				value and table is ascending, stop */
			break;
		} else {
			i++;
		}
	}

	if (i == 0)
		*output = pts[0].y;
	else if (i == tablesize)
		*output = pts[tablesize-1].y;
	else {
		/* result is between search_index and search_index-1 */
		/* interpolate linearly */
		*output = (((int32_t) ((pts[i].y - pts[i-1].y)*
			(input - pts[i-1].x))/
			(pts[i].x - pts[i-1].x))+
			pts[i-1].y);
	}

	return 0;
}

#ifndef CONFIG_BATTERY_SH
static int32_t pm8xxx_adc_map_batt_therm(const struct pm8xxx_adc_map_pt *pts,
		uint32_t tablesize, int32_t input, int64_t *output)
{
	bool descending = 1;
	uint32_t i = 0;

	if ((pts == NULL) || (output == NULL))
		return -EINVAL;

	/* Check if table is descending or ascending */
	if (tablesize > 1) {
		if (pts[0].y < pts[1].y)
			descending = 0;
	}

	while (i < tablesize) {
		if ((descending == 1) && (pts[i].y < input)) {
			/* table entry is less than measured
				value and table is descending, stop */
			break;
		} else if ((descending == 0) && (pts[i].y > input)) {
			/* table entry is greater than measured
				value and table is ascending, stop */
			break;
		} else {
			i++;
		}
	}

	if (i == 0) {
		*output = pts[0].x;
	} else if (i == tablesize) {
		*output = pts[tablesize-1].x;
	} else {
		/* result is between search_index and search_index-1 */
		/* interpolate linearly */
		*output = (((int32_t) ((pts[i].x - pts[i-1].x)*
			(input - pts[i-1].y))/
			(pts[i].y - pts[i-1].y))+
			pts[i-1].x);
	}

	return 0;
}
#endif /* CONFIG_BATTERY_SH */

int32_t pm8xxx_adc_scale_default(int32_t adc_code,
		const struct pm8xxx_adc_properties *adc_properties,
		const struct pm8xxx_adc_chan_properties *chan_properties,
		struct pm8xxx_adc_chan_result *adc_chan_result)
{
	bool negative_rawfromoffset = 0, negative_offset = 0;
	int64_t scale_voltage = 0;

	if (!chan_properties || !chan_properties->offset_gain_numerator ||
		!chan_properties->offset_gain_denominator || !adc_properties
		|| !adc_chan_result)
		return -EINVAL;

	scale_voltage = (adc_code -
		chan_properties->adc_graph[ADC_CALIB_ABSOLUTE].adc_gnd)
		* chan_properties->adc_graph[ADC_CALIB_ABSOLUTE].dx;
	if (scale_voltage < 0) {
		negative_offset = 1;
		scale_voltage = -scale_voltage;
	}
	do_div(scale_voltage,
		chan_properties->adc_graph[ADC_CALIB_ABSOLUTE].dy);
	if (negative_offset)
		scale_voltage = -scale_voltage;
	scale_voltage += chan_properties->adc_graph[ADC_CALIB_ABSOLUTE].dx;

	if (scale_voltage < 0) {
		if (adc_properties->bipolar) {
			scale_voltage = -scale_voltage;
			negative_rawfromoffset = 1;
		} else {
			scale_voltage = 0;
		}
	}

	adc_chan_result->measurement = scale_voltage *
				chan_properties->offset_gain_denominator;

	/* do_div only perform positive integer division! */
	do_div(adc_chan_result->measurement,
				chan_properties->offset_gain_numerator);

	if (negative_rawfromoffset)
		adc_chan_result->measurement = -adc_chan_result->measurement;

	/* Note: adc_chan_result->measurement is in the unit of
	 * adc_properties.adc_reference. For generic channel processing,
	 * channel measurement is a scale/ratio relative to the adc
	 * reference input */
	adc_chan_result->physical = adc_chan_result->measurement;

	return 0;
}
EXPORT_SYMBOL_GPL(pm8xxx_adc_scale_default);

static int64_t pm8xxx_adc_scale_ratiometric_calib(int32_t adc_code,
		const struct pm8xxx_adc_properties *adc_properties,
		const struct pm8xxx_adc_chan_properties *chan_properties)
{
	int64_t adc_voltage = 0;
	bool negative_offset = 0;

	if (!chan_properties || !chan_properties->offset_gain_numerator ||
		!chan_properties->offset_gain_denominator || !adc_properties)
		return -EINVAL;

	adc_voltage = (adc_code -
		chan_properties->adc_graph[ADC_CALIB_RATIOMETRIC].adc_gnd)
		* adc_properties->adc_vdd_reference;
	if (adc_voltage < 0) {
		negative_offset = 1;
		adc_voltage = -adc_voltage;
	}
	do_div(adc_voltage,
		chan_properties->adc_graph[ADC_CALIB_RATIOMETRIC].dy);
	if (negative_offset)
		adc_voltage = -adc_voltage;

	return adc_voltage;
}

#ifdef CONFIG_BATTERY_SH
static void pm8xxx_adc_scale_get_batt_therm_table( const struct pm8xxx_adc_map_pt **ptr_table, int *ptr_table_size )
{
	sharp_smem_common_type *sharp_smem;
	uint32_t hw_rev;
	int table_type = 0;

	sharp_smem = sh_smem_get_common_address();
	hw_rev = sharp_smem->sh_hw_revision;

#if defined(CONFIG_PM_BATT_THERM_TYPE1)
	switch (hw_rev)
	{
	case 0:
	case 4:
	case 5:
		table_type = 3;
		break;
	default:
		table_type = 2;
		break;
	}
#else
	switch (hw_rev)
	{
	case 0:
	case 4:
		/* none */
		break;
	default:
		table_type = 2;
		break;
	}
#endif

	if (table_type == 1)
	{
		*ptr_table = adcmap_batttherm_custom_1;
		*ptr_table_size = ARRAY_SIZE(adcmap_batttherm_custom_1);
	}
	else
	if (table_type == 2)
	{
		*ptr_table = adcmap_batttherm_custom_2;
		*ptr_table_size = ARRAY_SIZE(adcmap_batttherm_custom_2);
	}
	else
	if (table_type == 3)
	{
		*ptr_table = adcmap_batttherm_custom_3;
		*ptr_table_size = ARRAY_SIZE(adcmap_batttherm_custom_3);
	}
	else
	{
		*ptr_table = adcmap_batttherm;
		*ptr_table_size = ARRAY_SIZE(adcmap_batttherm);
	}
}

static void pm8xxx_adc_scale_get_pa_therm_table( const struct pm8xxx_adc_map_pt **ptr_table, int *ptr_table_size )
{
	sharp_smem_common_type *sharp_smem;
	uint32_t hw_rev;
	int table_type = 0;

	sharp_smem = sh_smem_get_common_address();
	hw_rev = sharp_smem->sh_hw_revision;

	switch (hw_rev)
	{
	case 0:
	case 4:
		/* none */
		break;
	default:
		table_type = 2;
		break;
	}

	if (table_type == 1)
	{
		*ptr_table = adcmap_pa_therm_custom_1;
		*ptr_table_size = ARRAY_SIZE(adcmap_pa_therm_custom_1);
	}
	else
	if (table_type == 2)
	{
		*ptr_table = adcmap_pa_therm_custom_2;
		*ptr_table_size = ARRAY_SIZE(adcmap_pa_therm_custom_2);
	}
	else
	{
		*ptr_table = adcmap_pa_therm;
		*ptr_table_size = ARRAY_SIZE(adcmap_pa_therm);
	}
}
#endif /* CONFIG_BATTERY_SH */

int32_t pm8xxx_adc_scale_batt_therm(int32_t adc_code,
		const struct pm8xxx_adc_properties *adc_properties,
		const struct pm8xxx_adc_chan_properties *chan_properties,
		struct pm8xxx_adc_chan_result *adc_chan_result)
{
#ifdef CONFIG_BATTERY_SH
	const struct pm8xxx_adc_map_pt *batt_therm_table_ptr;
	int batt_therm_table_size;

	if(pm8xxx_adc_scale_default(adc_code, adc_properties, chan_properties, adc_chan_result))
	{
		return -EINVAL;
	}

	adc_chan_result->microvolts = (int32_t)adc_chan_result->physical;

	pm8xxx_adc_scale_get_batt_therm_table(&batt_therm_table_ptr, &batt_therm_table_size);

	return pm8xxx_adc_map_linear(
			batt_therm_table_ptr,
			batt_therm_table_size,
			(int32_t)adc_chan_result->microvolts / 1000,
			&adc_chan_result->physical);
#else  /* CONFIG_BATTERY_SH */
	int64_t bat_voltage = 0;

	bat_voltage = pm8xxx_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);

	return pm8xxx_adc_map_batt_therm(
			adcmap_btm_threshold,
			ARRAY_SIZE(adcmap_btm_threshold),
			bat_voltage,
			&adc_chan_result->physical);
#endif /* CONFIG_BATTERY_SH */
}
EXPORT_SYMBOL_GPL(pm8xxx_adc_scale_batt_therm);

#ifdef CONFIG_BATTERY_SH
int32_t pm8xxx_adc_scale_cam_temp(int32_t adc_code,
		const struct pm8xxx_adc_properties *adc_properties,
		const struct pm8xxx_adc_chan_properties *chan_properties,
		struct pm8xxx_adc_chan_result *adc_chan_result)
{
	if(pm8xxx_adc_scale_default(adc_code, adc_properties, chan_properties, adc_chan_result))
	{
		return -EINVAL;
	}

	adc_chan_result->microvolts = (int32_t)adc_chan_result->physical;

	return pm8xxx_adc_map_linear(
			adcmap_camtemp,
			ARRAY_SIZE(adcmap_camtemp),
			(int32_t)adc_chan_result->microvolts / 1000,
			&adc_chan_result->physical);
}
EXPORT_SYMBOL_GPL(pm8xxx_adc_scale_cam_temp);

#ifdef CONFIG_PM_MPP_12_USE_BACKLIGHT_THERM
int32_t pm8xxx_adc_scale_backlight_temp(int32_t adc_code,
		const struct pm8xxx_adc_properties *adc_properties,
		const struct pm8xxx_adc_chan_properties *chan_properties,
		struct pm8xxx_adc_chan_result *adc_chan_result)
{
	if(pm8xxx_adc_scale_default(adc_code, adc_properties, chan_properties, adc_chan_result))
	{
		return -EINVAL;
	}

	adc_chan_result->microvolts = (int32_t)adc_chan_result->physical;

	return pm8xxx_adc_map_linear(
			adcmap_backlight_temp,
			ARRAY_SIZE(adcmap_backlight_temp),
			(int32_t)adc_chan_result->microvolts / 1000,
			&adc_chan_result->physical);
}
EXPORT_SYMBOL_GPL(pm8xxx_adc_scale_backlight_temp);
#endif /* CONFIG_PM_MPP_12_USE_BACKLIGHT_THERM */
#endif /* CONFIG_BATTERY_SH */

int32_t pm8xxx_adc_scale_pa_therm(int32_t adc_code,
		const struct pm8xxx_adc_properties *adc_properties,
		const struct pm8xxx_adc_chan_properties *chan_properties,
		struct pm8xxx_adc_chan_result *adc_chan_result)
{
#ifdef CONFIG_BATTERY_SH
	const struct pm8xxx_adc_map_pt *pa_therm_table_ptr;
	int pa_therm_table_size;

	if(pm8xxx_adc_scale_default(adc_code, adc_properties, chan_properties, adc_chan_result))
	{
		return -EINVAL;
	}

	adc_chan_result->microvolts = (int32_t)adc_chan_result->physical;

	pm8xxx_adc_scale_get_pa_therm_table(&pa_therm_table_ptr, &pa_therm_table_size);

	return pm8xxx_adc_map_linear(
			pa_therm_table_ptr,
			pa_therm_table_size,
			(int32_t)adc_chan_result->microvolts / 1000,
			&adc_chan_result->physical);
#else  /* CONFIG_BATTERY_SH */
	int64_t pa_voltage = 0;

	pa_voltage = pm8xxx_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);

	return pm8xxx_adc_map_linear(
			adcmap_pa_therm,
			ARRAY_SIZE(adcmap_pa_therm),
			pa_voltage,
			&adc_chan_result->physical);
#endif /* CONFIG_BATTERY_SH */
}
EXPORT_SYMBOL_GPL(pm8xxx_adc_scale_pa_therm);

int32_t pm8xxx_adc_scale_batt_id(int32_t adc_code,
		const struct pm8xxx_adc_properties *adc_properties,
		const struct pm8xxx_adc_chan_properties *chan_properties,
		struct pm8xxx_adc_chan_result *adc_chan_result)
{
	int64_t batt_id_voltage = 0;

	batt_id_voltage = pm8xxx_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);
	adc_chan_result->physical = batt_id_voltage;
	adc_chan_result->physical = adc_chan_result->measurement;

	return 0;
}
EXPORT_SYMBOL_GPL(pm8xxx_adc_scale_batt_id);

int32_t pm8xxx_adc_scale_pmic_therm(int32_t adc_code,
		const struct pm8xxx_adc_properties *adc_properties,
		const struct pm8xxx_adc_chan_properties *chan_properties,
		struct pm8xxx_adc_chan_result *adc_chan_result)
{
	int64_t pmic_voltage = 0;
	bool negative_offset = 0;

	if (!chan_properties || !chan_properties->offset_gain_numerator ||
		!chan_properties->offset_gain_denominator || !adc_properties
		|| !adc_chan_result)
		return -EINVAL;

	pmic_voltage = (adc_code -
		chan_properties->adc_graph[ADC_CALIB_ABSOLUTE].adc_gnd)
		* chan_properties->adc_graph[ADC_CALIB_ABSOLUTE].dx;
	if (pmic_voltage < 0) {
		negative_offset = 1;
		pmic_voltage = -pmic_voltage;
	}
	do_div(pmic_voltage,
		chan_properties->adc_graph[ADC_CALIB_ABSOLUTE].dy);
	if (negative_offset)
		pmic_voltage = -pmic_voltage;
	pmic_voltage += chan_properties->adc_graph[ADC_CALIB_ABSOLUTE].dx;

	if (pmic_voltage > 0) {
		/* 2mV/K */
		adc_chan_result->measurement = pmic_voltage*
			chan_properties->offset_gain_denominator;

		do_div(adc_chan_result->measurement,
			chan_properties->offset_gain_numerator * 2);
	} else {
		adc_chan_result->measurement = 0;
	}
	/* Change to .001 deg C */
	adc_chan_result->measurement -= KELVINMIL_DEGMIL;
	adc_chan_result->physical = (int32_t)adc_chan_result->measurement;

	return 0;
}
EXPORT_SYMBOL_GPL(pm8xxx_adc_scale_pmic_therm);

/* Scales the ADC code to 0.001 degrees C using the map
 * table for the XO thermistor.
 */
int32_t pm8xxx_adc_tdkntcg_therm(int32_t adc_code,
		const struct pm8xxx_adc_properties *adc_properties,
		const struct pm8xxx_adc_chan_properties *chan_properties,
		struct pm8xxx_adc_chan_result *adc_chan_result)
{
	int64_t xo_thm = 0;

	if (!chan_properties || !chan_properties->offset_gain_numerator ||
		!chan_properties->offset_gain_denominator || !adc_properties
		|| !adc_chan_result)
		return -EINVAL;

	xo_thm = pm8xxx_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);
#ifdef CONFIG_BATTERY_SH
	if(xo_thm == -EINVAL)
	{
		return -EINVAL;
	}
	adc_chan_result->measurement = xo_thm;
	adc_chan_result->microvolts = (int32_t)adc_chan_result->measurement * 1000;

	xo_thm = adc_code -
		chan_properties->adc_graph[ADC_CALIB_RATIOMETRIC].adc_gnd;
	if (xo_thm < 0)
	{
		xo_thm = 0;
	}
	xo_thm <<= 14;
	if (adc_code < chan_properties->adc_graph[ADC_CALIB_RATIOMETRIC].adc_vref)
	{
		do_div(xo_thm,
			(chan_properties->adc_graph[ADC_CALIB_RATIOMETRIC].adc_vref
			- adc_code));
	}
#else  /* CONFIG_BATTERY_SH */
	xo_thm <<= 4;
#endif /* CONFIG_BATTERY_SH */
	pm8xxx_adc_map_linear(adcmap_ntcg_104ef_104fb,
		ARRAY_SIZE(adcmap_ntcg_104ef_104fb),
		xo_thm, &adc_chan_result->physical);

	return 0;
}
EXPORT_SYMBOL_GPL(pm8xxx_adc_tdkntcg_therm);

#ifdef CONFIG_BATTERY_SH
#define XOADC_CALIB_ADJUST (-1)

static int XoAdc_admin = 0;
static int XoAdc_admax = 0;
static int XoAdc_volmin = 0;
static int XoAdc_volmax = 0;
static bool XoAdc_calflg = false;
static bool XoAdc_adcflg = false;

static void pm8xxx_adc_scale_vbatt_calib_type_0(int32_t adc_code,
		struct pm8xxx_adc_chan_result *adc_chan_result)
{
	static int scale_x_10000 = 0;
	static int64_t offset = 0;
	bool negative_offset = 0;

	if (XoAdc_calflg == false)
	{
		if (XoAdc_admin != XoAdc_admax)
		{
			scale_x_10000 = ((XoAdc_volmax - XoAdc_volmin) * 10000) / (XoAdc_admax - XoAdc_admin);
			offset = (XoAdc_volmin * XoAdc_admax) - (XoAdc_volmax * XoAdc_admin);
			offset *= 1000;
			if (offset < 0)
			{
				negative_offset = 1;
				offset = -offset;
			}
			do_div(offset, XoAdc_admax - XoAdc_admin);
			if (negative_offset)
			{
				offset = -offset;
			}
			pr_debug("calib: scale = %d offset = %lld\n", scale_x_10000, offset);
			XoAdc_calflg = true;
		}
	}

	if (XoAdc_calflg == true)
	{
		adc_chan_result->physical = ((uint32_t)(adc_code * scale_x_10000)) / 10 + offset;
	}
}

static void pm8xxx_adc_scale_vbatt_calib_type_1(int32_t adc_code,
		const struct pm8xxx_adc_chan_properties *chan_properties,
		struct pm8xxx_adc_chan_result *adc_chan_result)
{
	static int calc_gain = 0;
	static int calc_offset = 0;
	int calc_data;

	if (XoAdc_calflg == false)
	{
		calc_data = chan_properties->adc_graph[ADC_CALIB_ABSOLUTE].dy * (1 << 14);
		calc_gain = calc_data / 625;
		calc_offset = chan_properties->adc_graph[ADC_CALIB_ABSOLUTE].adc_gnd * (1 << 14);
		calc_offset -= calc_data;
		pr_debug("calib: gain = %d offset = %d\n", calc_gain, calc_offset);

		if ((calc_gain != 0) &&
			(XoAdc_admin != XoAdc_admax) && (XoAdc_volmin != XoAdc_volmax))
		{
			XoAdc_calflg = true;
		}
		if (calc_gain == 0)
		{
			XoAdc_adcflg = false;
		}
	}

	if (XoAdc_calflg == false)
	{
		if (XoAdc_adcflg == true)
		{
			calc_data = ((adc_code * (1 << 14) - calc_offset) * (1 << 3)) / calc_gain;
			adc_chan_result->adc_code = calc_data;
			XoAdc_adcflg = false;
		}
	}
	else
	{
		if ((calc_gain != 0) && (XoAdc_admax != XoAdc_admin))
		{
			calc_data = ((adc_code * (1 << 14) - calc_offset) * (1 << 3)) / calc_gain;
			calc_data = (XoAdc_volmax - XoAdc_volmin) * (calc_data - XoAdc_admax) * 1000;
			calc_data /= XoAdc_admax - XoAdc_admin;
			calc_data += XoAdc_volmax * 1000;
			adc_chan_result->physical = calc_data;
		}
	}
}

int32_t pm8xxx_adc_scale_vbatt(int32_t adc_code,
		const struct pm8xxx_adc_properties *adc_properties,
		const struct pm8xxx_adc_chan_properties *chan_properties,
		struct pm8xxx_adc_chan_result *adc_chan_result)
{
	pm8xxx_adc_scale_default(adc_code, adc_properties, chan_properties, adc_chan_result);

	if (((XoAdc_volmax == XOADC_CALIB_ADJUST) &&
		 (XoAdc_volmin == XOADC_CALIB_ADJUST) &&
		 (XoAdc_admax == XOADC_CALIB_ADJUST) &&
		 (XoAdc_admin == XOADC_CALIB_ADJUST)) ||
		((XoAdc_volmax >= 3700) && (XoAdc_volmax <= 4300) &&
		 (XoAdc_volmin >= 3000) && (XoAdc_volmin <= 3600) &&
		 (XoAdc_admax < 20000) && (XoAdc_admin < 20000)))
	{
		pm8xxx_adc_scale_vbatt_calib_type_1(adc_code, chan_properties, adc_chan_result);
	}
	else
	{
		pm8xxx_adc_scale_vbatt_calib_type_0(adc_code, adc_chan_result);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(pm8xxx_adc_scale_vbatt);

void pm8xxx_adc_set_vbatt_calibration_data(int admin, int admax, int volmin, int volmax)
{
	XoAdc_admin = admin;
	XoAdc_admax = admax;
	XoAdc_volmin = volmin;
	XoAdc_volmax = volmax;
}
EXPORT_SYMBOL_GPL(pm8xxx_adc_set_vbatt_calibration_data);

void pm8xxx_adc_refresh_vbatt_calibration_data(void)
{
	XoAdc_calflg = false;
	if ((XoAdc_volmax == XOADC_CALIB_ADJUST) &&
		(XoAdc_volmin == XOADC_CALIB_ADJUST) &&
		(XoAdc_admax == XOADC_CALIB_ADJUST) &&
		(XoAdc_admin == XOADC_CALIB_ADJUST))
	{
		XoAdc_adcflg = true;
	}
}
EXPORT_SYMBOL_GPL(pm8xxx_adc_refresh_vbatt_calibration_data);
#endif /* CONFIG_BATTERY_SH */

int32_t pm8xxx_adc_batt_scaler(struct pm8xxx_adc_arb_btm_param *btm_param,
		const struct pm8xxx_adc_properties *adc_properties,
		const struct pm8xxx_adc_chan_properties *chan_properties)
{
	int rc;

	rc = pm8xxx_adc_map_linear(
		adcmap_btm_threshold,
		ARRAY_SIZE(adcmap_btm_threshold),
		(btm_param->low_thr_temp),
		&btm_param->low_thr_voltage);
	if (rc)
		return rc;

	btm_param->low_thr_voltage *=
		chan_properties->adc_graph[ADC_CALIB_RATIOMETRIC].dy;
	do_div(btm_param->low_thr_voltage, adc_properties->adc_vdd_reference);
	btm_param->low_thr_voltage +=
		chan_properties->adc_graph[ADC_CALIB_RATIOMETRIC].adc_gnd;

	rc = pm8xxx_adc_map_linear(
		adcmap_btm_threshold,
		ARRAY_SIZE(adcmap_btm_threshold),
		(btm_param->high_thr_temp),
		&btm_param->high_thr_voltage);
	if (rc)
		return rc;

	btm_param->high_thr_voltage *=
		chan_properties->adc_graph[ADC_CALIB_RATIOMETRIC].dy;
	do_div(btm_param->high_thr_voltage, adc_properties->adc_vdd_reference);
	btm_param->high_thr_voltage +=
		chan_properties->adc_graph[ADC_CALIB_RATIOMETRIC].adc_gnd;


	return rc;
}
EXPORT_SYMBOL_GPL(pm8xxx_adc_batt_scaler);
