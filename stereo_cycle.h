// Строит облако точек в цикле, на каждой итерации позволяя пользователю менять параметры алгоритма
// * Пользователь сам может выбрать редактируемый параметр
// * Свойства алгоритмов вынесены в структуры
//   * Существует структура, содеражащая общие настройки алгоритмов (COMMON)
//   * Индивидуальные настройки для конкретного алгоритма содержатся в соответствующей его имни структуре (ACSOSM_args, BBSM_args)
//   * В структурах заданы значения по - умолчанию для всех параметров того или иного алгоритма


#include <pcl/stereo/stereo_matching.h>
//#include <pcl/visualization/pcl_visualizer.h> Uncomment this if you have working VTK
//#include <pcl/visualization/image_viewer.h>

#include <pcl/io/pcd_io.h>

// Settings for AdaptiveCostSOStereoMatching algorithm
struct ACSOSM_args
{
	// Default arguments
	ACSOSM_args()
	:radius(5),
	gamma_s(10),
	gamma_c(25),
	smoothness_weak(20),
	smoothness_strong(100)
	{}
	// radius (half length) of the column used for cost aggregation; 
	// the total column length is equal to 2*radius + 1
	int radius;

	// spatial bandwith used for cost aggregation based on adaptive weights

	int gamma_s;

	//  color bandwith used for cost aggregation based on adaptive weights
	int gamma_c;

	// "weak" smoothness penalty used within 2-pass Scanline Optimization
	int smoothness_weak;

	// "strong" smoothness penalty used within 2-pass Scanline Optimization
	int smoothness_strong;
};

// Settings for BlockBasedStereoMatching algorithm
struct BBSM_args
{
	// Default arguments
	BBSM_args()
	:radius(5)
	{}
	// setter for the radius of the squared window
	// radius of the squared window used to compute the block-based stereo algorithm 
	// the window side is equal to 2*radius + 1
	int radius;
};

struct Common_args
{
	// Default arguments
	Common_args()
	:max_disparity(60),
	x_offset(0),
	ratio_filter(20),
	peak_filter(0),
	is_pre_processing(true),
	is_left_right_check(true),
	left_to_right_check_treshold(1),
	median_filter_radius(4)
	{}

	// number of disparity candidates (disparity range); 
	// || > 0
	int max_disparity;

	// number of pixels to shift the disparity range over the target image
	// || >= 0
	int x_offset; // horizontal offset

	// value of the ratio filter;
	// it is a number in the range[0, 100]
	// (0: no filtering action; 100: all disparities are filtered)
	// || [0; 100]
	int ratio_filter;

	// value of the peak filter; it is a number in the range 
	// || [0, inf] (0: no filtering action)
	int peak_filter;

	// setting the boolean to true activates the pre-processing step for both stereo images
	bool is_pre_processing;

	// setter for the left-right consistency check stage,
	// that eliminates inconsistent/wrong disparity values from the disparity map at approx.
	// setting the boolean to true activates the left-right consistency check
	bool is_left_right_check;

	// sets the value of the left-right consistency check threshold 
	// only has some influence if the left-right check is active typically has either the value 
	// 0 ("strong" consistency check, more points being filtered) 
	// 1 ("weak" consistency check, less points being filtered)
	int left_to_right_check_treshold;


	// median filter applied on the previously computed disparity map
	// this radius of the squared window used to compute the median filter; 
	// the window side is equal to 2*radius + 1
	int median_filter_radius;
};

// Prototypes
void stereo_cycle(pcl::PointCloud<pcl::RGB>::Ptr leftCloud, pcl::PointCloud<pcl::RGB>::Ptr rightCloud, double f, double u0, double v0, double dist);
void ACSOSM_preset(pcl::AdaptiveCostSOStereoMatching & acsosm, Common_args & comm_a, ACSOSM_args & acsosm_a);
void BBSM_preset(pcl::BlockBasedStereoMatching & bbsm, Common_args & comm_a, BBSM_args & bbsm_a);
void print_ACSOSM_properties_list();
void print_BBSM_properties_list();
void print_Common_properties_list();


// VV==MAIN==VV
// Содержит цикл, в котором
// 1 - Изменяются параметры алгоритма
// 2 - Пересчитывается облако точек в соответствии с выбранным алгоритмом
void stereo_cycle(pcl::PointCloud<pcl::RGB>::Ptr leftCloud, pcl::PointCloud<pcl::RGB>::Ptr rightCloud, double f, double u0, double v0, double dist)
{
	// Presets
	setlocale(LC_ALL, "Russian");

	// Algorithm objects
	pcl::AdaptiveCostSOStereoMatching acsosm;
	pcl::BlockBasedStereoMatching bbsm;
	// Algorithm settings
	Common_args comm_a;
	ACSOSM_args acsosm_a;
	BBSM_args bbsm_a;


	// Output point cloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	// Staff variables
	enum { CONTINUE_CMD = 'y' };
	char command; // Continue? 
	int alg; // 1 - ACSOSM, 2 - BBSM

	// INPUT
	printf("Выберите алгоритм (1 - для ACSOSM, 2 - для BBSM): ");
	scanf("%c", &alg);

	// CALCULATIONS
	do
	{
		switch (alg)
		{
		case 1:
			ACSOSM_preset(acsosm, comm_a, acsosm_a);

			acsosm.compute(*leftCloud, *rightCloud);
			acsosm.medianFilter(comm_a.median_filter_radius);

			acsosm.getPointCloud(u0, v0, f, dist, outCloud, leftCloud);
			break;

		case 2:
			BBSM_preset(bbsm, comm_a, bbsm_a);

			bbsm.compute(*leftCloud, *rightCloud);
			bbsm.medianFilter(comm_a.median_filter_radius);

			bbsm.getPointCloud(u0, v0, f, dist, outCloud, rightCloud);
			break;
		}

		// Now you can visualize out_cloud
		// visualize(outCloud)


		scanf("%c", &command);
	} while (command != CONTINUE_CMD);

}

// Функции настройки алгоритмов
// SETTINGS
// Обновить настройки алгоритма ACSOSM по введенным данным
// Позволяет выбрать редактируемое свойство алгоритма
// * acsosm[in/out]   : объект алгоритма
// * comm_a[in/out]   : общие настройки алгоритмов BBSM и ACSOSM
// * acsosm_a[in/out] : настройки ACSOSM алгоритма
void ACSOSM_preset(pcl::AdaptiveCostSOStereoMatching & acsosm, Common_args & comm_a, ACSOSM_args & acsosm_a)
{
	print_Common_properties_list();
	print_ACSOSM_properties_list();
	printf("Введите номер свойства, которые вы хотите редактировать (-1 для завершения):");
	int cmd; // ^

	// exit from cycle when break in switch
	while (true)
	{
		scanf("%d", &cmd);
		switch (cmd)
		{
		case 1:
			printf("1)MaxDisparity[int] (%d): ", comm_a.max_disparity);
			scanf("%d", &comm_a.max_disparity);
			acsosm.setMaxDisparity(comm_a.max_disparity);
			break;

		case 2:
			printf("2)XOffset[int] (%d): ", comm_a.x_offset);
			scanf("%d", &comm_a.x_offset);
			acsosm.setXOffset(comm_a.x_offset);
			break;

		case 3:
			printf("3)RatioFilter[int] (%d): ", comm_a.ratio_filter);
			scanf("%d", &comm_a.ratio_filter);
			acsosm.setRatioFilter(comm_a.ratio_filter);
			break;

		case 4:
			printf("4)PeakFilter[int] (%d): ", comm_a.peak_filter);
			scanf("%d", &comm_a.peak_filter);
			acsosm.setPeakFilter(comm_a.peak_filter);
			break;

		case 5:
			printf("5)PreProcessing[bool](0 = false, 1 = true) (%d): ", comm_a.is_pre_processing);
			scanf("%d", &comm_a.is_pre_processing);
			acsosm.setPreProcessing(comm_a.is_pre_processing);
			break;

		case 6:
			printf("6)LeftRightCheck[bool](0 = false, 1 = true) (%d): ", comm_a.is_left_right_check);
			scanf("%d", &comm_a.is_left_right_check);
			acsosm.setLeftRightCheck(comm_a.is_left_right_check);
			break;

		case 7:
			printf("7)LeftRightCheckTreshold[int] (%d): ", comm_a.left_to_right_check_treshold);
			scanf("%d", &comm_a.left_to_right_check_treshold);
			acsosm.setLeftRightCheckThreshold(comm_a.left_to_right_check_treshold);
			break;

		case 8:
			printf("8)MedianFilterRadius[int] (%d): ", comm_a.median_filter_radius);
			scanf("%d", &comm_a.median_filter_radius);
			// Not YET!
			break;

		case 9:
			printf("9)Radius[int] (%d): ", acsosm_a.radius);
			scanf("%d", &acsosm_a.radius);
			acsosm.setRadius(acsosm_a.radius);
			break;

		case 10:
			printf("10)GammaS[int] (%d): ", acsosm_a.gamma_s);
			scanf("%d", &acsosm_a.gamma_s);
			acsosm.setGammaS(acsosm_a.gamma_s);
			break;

		case 11:
			printf("11)GammaC[int] (%d): ", acsosm_a.gamma_c);
			scanf("%d", &acsosm_a.gamma_c);
			acsosm.setGammaC(acsosm_a.gamma_c);
			break;

		case 12:
			printf("12)SmoothnessWeak[int] (%d): ", acsosm_a.smoothness_weak);
			scanf("%d", &acsosm_a.smoothness_weak);
			acsosm.setSmoothWeak(acsosm_a.smoothness_weak);
			break;

		case 13:
			printf("13)SmoothnessStrong[int] (%d): ", acsosm_a.smoothness_strong);
			scanf("%d", &acsosm_a.smoothness_strong);
			acsosm.setSmoothStrong(acsosm_a.smoothness_strong);
			break;

		default:
			break; // exit_cycle
		}
	}
}


// Обновить настройки алгоритма BBSM по введенным данным
// Позволяет выбрать редактируемое свойство алгоритма BBSM
// * bbsm[in/out]   : объект алгоритма
// * comm_a[in/out] : общие настройки алгоритмов BBSM и ACSOSM
// * bbsm_a[in/out] : настройки BBSM алгоритма
void BBSM_preset(pcl::BlockBasedStereoMatching & bbsm, Common_args & comm_a, BBSM_args & bbsm_a)
{
	print_Common_properties_list();
	print_BBSM_properties_list();

	printf("Введите номер свойства, которые вы хотите редактировать (-1 для завершения):");
	int cmd; // ^

	// exit from cycle when break in switch
	while (true)
	{
		scanf("%d", &cmd);
		switch (cmd)
		{
		case 1:
			printf("1)MaxDisparity[int] (%d): ", comm_a.max_disparity);
			scanf("%d", &comm_a.max_disparity);
			bbsm.setMaxDisparity(comm_a.max_disparity);
			break;

		case 2:
			printf("2)XOffset[int] (%d): ", comm_a.x_offset);
			scanf("%d", &comm_a.x_offset);
			bbsm.setXOffset(comm_a.x_offset);
			break;

		case 3:
			printf("3)RatioFilter[int] (%d): ", comm_a.ratio_filter);
			scanf("%d", &comm_a.ratio_filter);
			bbsm.setRatioFilter(comm_a.ratio_filter);
			break;

		case 4:
			printf("4)PeakFilter[int] (%d): ", comm_a.peak_filter);
			scanf("%d", &comm_a.peak_filter);
			bbsm.setPeakFilter(comm_a.peak_filter);
			break;

		case 5:
			printf("5)PreProcessing[bool](0 = false, 1 = true) (%d): ", comm_a.is_pre_processing);
			scanf("%d", &comm_a.is_pre_processing);
			bbsm.setPreProcessing(comm_a.is_pre_processing);
			break;

		case 6:
			printf("6)LeftRightCheck[bool](0 = false, 1 = true) (%d): ", comm_a.is_left_right_check);
			scanf("%d", &comm_a.is_left_right_check);
			bbsm.setLeftRightCheck(comm_a.is_left_right_check);
			break;

		case 7:
			printf("7)LeftRightCheckTreshold[int] (%d): ", comm_a.left_to_right_check_treshold);
			scanf("%d", &comm_a.left_to_right_check_treshold);
			bbsm.setLeftRightCheckThreshold(comm_a.left_to_right_check_treshold);
			break;

		case 8:
			printf("8)MedianFilterRadius[int] (%d): ", comm_a.median_filter_radius);
			scanf("%d", &comm_a.median_filter_radius);
			// Not YET!
			break;

		case 9:
			printf("9)Radius[int] (%d): ", bbsm_a.radius);
			scanf("%d", &bbsm_a.radius);
			bbsm.setRadius(bbsm_a.radius);
			break;

		default:
			break; // exit_cycle
		}
	}
}


// Функции печати, выводящие список свойств конкретного алгоритма на экран
// PRINT
void print_Common_properties_list()
{
	printf("%s\n%s\n%s\n%s\n%s\n%s\n%s\n%s\n",
		"1)MaxDisparity[int]: ",
		"2)XOffset[int]: ",
		"3)RatioFilter[int]: ",
		"4)PeakFilter[int]: ",
		"5)PreProcessing[bool](0 = false, 1 = true): ",
		"6)LeftRightCheck[bool](0 = false, 1 = true): ",
		"7)LeftRightCheckTreshold[int]: ",
		"8)MedianFilterRadius[int]: ");
}

void print_ACSOSM_properties_list()
{
	printf("%s\n%s\n%s\n%s\n%s\n",
		"9)Radius[int]: ",
		"10)GammaS[int]",
		"11)GammaC[int]",
		"12)SmoothnessWeak[int]",
		"13)SmoothnessStrong[int]");
}

void print_BBSM_properties_list()
{
	puts("9)Radius[int]: ");
}