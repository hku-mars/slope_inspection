#ifndef  AERODYNAMIC_HPP
#define  AERODYNAMIC_HPP

#include <Eigen/Dense>
#include <boost/math/tools/roots.hpp>


using namespace std;

#define print_line printf("%s -- %d\r\n", __FILE__, __LINE__);

#define TAILSITTER_MASS             2.475
#define TAILSITTER_SQUARE           0.225
#define TAILSITTER_AIR_DENSITY      1.225
#define DEG2RAD                     0.01745329252
#define PI                          3.141592653589793
#define GRAVITY                     9.8
#define INF                         1e6
#define EPSILON                     1e-6
#define VEL_MIN                     0.5
#define GAMMA_MIN                   (5.0 * DEG2RAD)

#define AERODYN_BREAK_INTERVAL      (5.0 * DEG2RAD)
#define AERODYN_PIECES              72
#define AERODYN_ORDER               3

#define ALPHA_SOLVER_STEPOUT        (10.0 * DEG2RAD)
#define ALPHA_SOLVER_STEPOUT_RATIO  1.5
#define ALPHA_SOLVER_SMALL_BOUND    (15.0 * DEG2RAD)
#define ALPHA_SOLVER_MAX_ITER       8
#define ALPHA_SOLVER_EPSILON        1e-4

#define QUAT_NED2ENU    {0.0, 0.707106781, 0.707106781, 0.0}
#define QUAT_FRD2FLU    {0.0, 1.0, 0.0, 0.0}


typedef double aerodyn_alpha_list_t[AERODYN_PIECES + 1];
typedef double aerodyn_coef_model_t[AERODYN_PIECES][AERODYN_ORDER + 1];

static const aerodyn_alpha_list_t aerodyn_alpha_list = {-3.14159265, -3.05432619, -2.96705973, -2.87979327, -2.79252680,
                                                        -2.70526034, -2.61799388, -2.53072742, -2.44346095, -2.35619449,
                                                        -2.26892803, -2.18166156, -2.09439510, -2.00712864, -1.91986218,
                                                        -1.83259571, -1.74532925, -1.65806279, -1.57079633, -1.48352986,
                                                        -1.39626340, -1.30899694, -1.22173048, -1.13446401, -1.04719755,
                                                        -0.95993109, -0.87266463, -0.78539816, -0.69813170, -0.61086524,
                                                        -0.52359878, -0.43633231, -0.34906585, -0.26179939, -0.17453293,
                                                        -0.08726646, 0.00000000, 0.08726646, 0.17453293, 0.26179939,
                                                        0.34906585, 0.43633231, 0.52359878, 0.61086524, 0.69813170,
                                                        0.78539816, 0.87266463, 0.95993109, 1.04719755, 1.13446401,
                                                        1.22173048, 1.30899694, 1.39626340, 1.48352986, 1.57079633,
                                                        1.65806279, 1.74532925, 1.83259571, 1.91986218, 2.00712864,
                                                        2.09439510, 2.18166156, 2.26892803, 2.35619449, 2.44346095,
                                                        2.53072742, 2.61799388, 2.70526034, 2.79252680, 2.87979327,
                                                        2.96705973, 3.05432619, 3.14159265};

static const aerodyn_coef_model_t aerodyn_CL_coef_model = {
        -4.00430713, -0.65205360, 1.60497869, -0.10027993,
        -6.29352188, -1.70037876, 1.39969024, 0.03215406,
        3.39767896, -3.34801893, 0.95913443, 0.13716845,
        8.15147844, -2.45850866, 0.45241931, 0.19763010,
        -3.03667185, -0.32445659, 0.20955978, 0.22380576,
        2.13549599, -1.11945543, 0.08355468, 0.23760434,
        0.21511091, -0.56038388, -0.06303895, 0.23778991,
        0.83081941, -0.50406798, -0.15592990, 0.22816411,
        0.51612206, -0.28655996, -0.22492520, 0.21127011,
        0.45441835, -0.15143952, -0.26314787, 0.18980240,
        0.28295732, -0.03247308, -0.27919727, 0.16598713,
        0.13685421, 0.04160498, -0.27840036, 0.14156332,
        -0.00411819, 0.07743333, -0.26801231, 0.11767610,
        -0.11489234, 0.07635519, -0.25459173, 0.09487456,
        -0.18808853, 0.04627644, -0.24389010, 0.07316237,
        -0.21648903, -0.00296502, -0.24011047, 0.05210636,
        -0.19901144, -0.05964171, -0.24557394, 0.03098631,
        -0.13910432, -0.11174279, -0.26053006, 0.00896949,
        -0.04462255, -0.14816022, -0.28321087, -0.01470946,
        0.07289746, -0.15984237, -0.31008917, -0.04058223,
        0.20052917, -0.14075786, -0.33632149, -0.06881144,
        0.32153458, -0.08825945, -0.35630702, -0.09909969,
        0.42968900, -0.00408189, -0.36436532, -0.13065180,
        0.48720805, 0.10841042, -0.35526094, -0.16219420,
        0.56905298, 0.23596119, -0.32520885, -0.19204719,
        0.39544093, 0.38493891, -0.27102509, -0.21825189,
        0.81551460, 0.48846511, -0.19480621, -0.23870901,
        -0.82387199, 0.70196633, -0.09092147, -0.25144722,
        2.85310551, 0.48627715, 0.01277233, -0.25458336,
        -8.82259044, 1.23321843, 0.16282663, -0.24786945,
        6.50193801, -1.07653035, 0.17650025, -0.23013190,
        18.40245773, 0.62567304, 0.13715552, -0.21860659,
        -14.67937221, 5.44342521, 0.66678426, -0.18964298,
        11.50452690, 1.60037455, 1.28147175, -0.09975652,
        -24.11394414, 4.61225265, 1.82362575, 0.03190612,
        -18.05447529, -1.70076316, 2.07770114, 0.21014635,
        1.63251585, -6.42741374, 1.36838389, 0.36650942,
        23.53155742, -6.00002209, 0.28388553, 0.43806081,
        -2.00585825, 0.16052523, -0.22570671, 0.43278013,
        3.53374372, -0.36460723, -0.24351622, 0.41297294,
        1.64200804, 0.56052472, -0.22641920, 0.39129392,
        -4.03367082, 0.99040142, -0.09107536, 0.37689500,
        -0.01687967, -0.06561113, -0.01037218, 0.37380884,
        -2.94820569, -0.07003022, -0.02220912, 0.37239282,
        -4.51535221, -0.84186867, -0.10178731, 0.36796211,
        4.17917288, -2.02398511, -0.35188023, 0.34966751,
        2.15972687, -0.92988021, -0.60965361, 0.30632401,
        3.74629337, -0.36446504, -0.72260654, 0.24747554,
        -1.10992383, 0.61631227, -0.70062873, 0.18413035,
        -3.37874715, 0.32573490, -0.61841960, 0.12694482,
        2.08171485, -0.55881904, -0.63876003, 0.07321272,
        -0.47258920, -0.01382737, -0.68873286, 0.01459819,
        0.42791663, -0.13755093, -0.70194311, -0.04592446,
        0.08323123, -0.02552262, -0.71617396, -0.10794368,
        0.19436433, -0.00373273, -0.71872697, -0.17058070,
        0.18694879, 0.04715173, -0.71493795, -0.23320072,
        0.24093785, 0.09609481, -0.70243733, -0.29510750,
        0.29121812, 0.15917219, -0.68016108, -0.35551480,
        0.34601152, 0.23541292, -0.64572703, -0.41346435,
        0.39308459, 0.32599852, -0.59673464, -0.46779194,
        0.41742368, 0.42890783, -0.53085663, -0.51712301,
        0.43810001, 0.53818909, -0.44646151, -0.55990526,
        0.36569522, 0.65288340, -0.34252083, -0.59447668,
        0.46671904, 0.74862219, -0.22021639, -0.61915224,
        -0.07050214, 0.87080895, -0.07889436, -0.63235850,
        1.09819267, 0.85235153, 0.07147976, -0.63265859,
        -2.15350299, 1.13985770, 0.24533281, -0.61919995,
        3.37629240, 0.57607194, 0.39507592, -0.59054127,
        7.18985272, 1.45998322, 0.57275525, -0.54943356,
        -6.68207190, 3.34228226, 0.99183197, -0.48355465,
        -3.84973219, 1.59291993, 1.42250961, -0.37598877,
        -4.78513286, 0.58506240, 1.61257442, -0.24227904,
};

static const aerodyn_coef_model_t aerodyn_CD_coef_model = {
        -3.22246819, 1.11140659, 1.02830340, 0.39691184,
        -4.78686051, 0.26776639, 1.14865895, 0.49297053,
        1.88910578, -0.98543076, 1.08603092, 0.59206787,
        4.93067086, -0.49086403, 0.95719989, 0.68059291,
        -3.07073191, 0.79998258, 0.98417557, 0.76366300,
        0.45837586, -0.00393315, 1.05364399, 0.85360002,
        -0.92955463, 0.11606937, 1.06342972, 0.94582247,
        -0.47382552, -0.12728746, 1.06245076, 1.03889038,
        -0.61314350, -0.25133470, 1.02940974, 1.13032246,
        -0.52522513, -0.41185529, 0.97153550, 1.21783391,
        -0.47764641, -0.54935891, 0.88765374, 1.29913087,
        -0.39069922, -0.67440644, 0.78086006, 1.37209223,
        -0.29479258, -0.77669126, 0.65422790, 1.43483958,
        -0.18793349, -0.85386778, 0.51193478, 1.48582098,
        -0.07854698, -0.90306865, 0.35861315, 1.52386825,
        0.02801541, -0.92363220, 0.19920343, 1.54823369,
        0.12608482, -0.91629778, 0.03863925, 1.55860222,
        0.21119408, -0.88328885, -0.11840431, 1.55507992,
        0.28006566, -0.82799837, -0.26774229, 1.53816092,
        0.33077271, -0.75467735, -0.40585680, 1.50867655,
        0.36309102, -0.66808126, -0.53001592, 1.46773149,
        0.37723810, -0.57302425, -0.63832280, 1.41663245,
        0.37766183, -0.47426355, -0.72971590, 1.35681515,
        0.35940106, -0.37539192, -0.80386233, 1.28977468,
        0.35185428, -0.28130094, -0.86116959, 1.21700453,
        0.27460535, -0.18918570, -0.90222730, 1.13994491,
        0.39118041, -0.11729419, -0.92897271, 1.05995249,
        -0.10689529, -0.01488340, -0.94050738, 0.97825105,
        0.99319224, -0.04286852, -0.94554719, 0.89599192,
        -2.57447894, 0.21714860, -0.93033838, 0.81381094,
        1.11809193, -0.45684841, -0.95125614, 0.73256636,
        7.20152161, -0.16413262, -1.00544695, 0.64681755,
        -2.03772130, 1.72122133, -0.86956533, 0.56261174,
        -0.13679685, 1.18774714, -0.61570994, 0.49848149,
        -0.63621029, 1.15193381, -0.41153426, 0.45370496,
        -0.53948097, 0.98537434, -0.22501894, 0.42614149,
        -0.40347957, 0.84413855, -0.06536382, 0.41365041,
        -0.13044931, 0.73850785, 0.07274813, 0.41410669,
        -0.17155847, 0.70435630, 0.19866178, 0.42599252,
        -0.20082199, 0.65944240, 0.31767567, 0.44857900,
        -0.04265872, 0.60686733, 0.42818204, 0.48118991,
        -0.70027660, 0.59569930, 0.53312577, 0.52314905,
        -0.20819090, 0.41236731, 0.62109618, 0.57374418,
        0.61181740, 0.35786306, 0.68831146, 0.63094705,
        -0.89649372, 0.51803648, 0.76474812, 0.69414543,
        -0.79232071, 0.28333497, 0.83468097, 0.76423158,
        -1.02475609, 0.07590590, 0.86603065, 0.83870240,
        -0.96606132, -0.19237462, 0.85586684, 0.91417487,
        -0.88238951, -0.44528888, 0.80022020, 0.98675631,
        -0.71796402, -0.67629791, 0.70234329, 1.05261121,
        -0.51595466, -0.86426045, 0.56790421, 1.10827478,
        -0.28759380, -0.99933707, 0.40527464, 1.15090917,
        -0.05403905, -1.07462895, 0.22428697, 1.17847454,
        0.16720458, -1.08877634, 0.03549424, 1.18982759,
        0.35958024, -1.04500228, -0.15071307, 1.18474466,
        0.50987589, -0.95086439, -0.32488530, 1.16387328,
        0.60871519, -0.81737920, -0.47919366, 1.12861929,
        0.65197314, -0.65801793, -0.60794635, 1.08098159,
        0.63886191, -0.48733177, -0.70789697, 1.02335046,
        0.57904048, -0.32007811, -0.77835677, 0.95828812,
        0.46756521, -0.16848567, -0.82099200, 0.88831096,
        0.36455499, -0.04607738, -0.83971616, 0.81569353,
        0.13166651, 0.04936289, -0.83942945, 0.74230584,
        0.26407045, 0.08383310, -0.82780590, 0.66951523,
        -0.67957118, 0.15296659, -0.80714123, 0.59808945,
        1.48546196, -0.02494473, -0.79596922, 0.52836638,
        -4.11296833, 0.36394830, -0.76638557, 0.45970219,
        5.61629561, -0.71282429, -0.79683075, 0.39286069,
        14.76507799, 0.75751846, -0.79293044, 0.32162806,
        -7.02637063, 4.62300684, -0.32339103, 0.26801311,
        -2.57685441, 2.78350731, 0.32294926, 0.27032860,
        -3.88969288, 2.10888840, 0.74989132, 0.31799636,
};

static const aerodyn_coef_model_t aerodyn_CY_coef_model = {
        -0.09121652, 0.02828133, 0.04831226, -0.02598329,
        -0.04914800, 0.00440090, 0.05116433, -0.02161249,
        0.00759002, -0.00846601, 0.05080958, -0.01714671,
        -0.01945784, -0.00647895, 0.04950539, -0.01277217,
        -0.00738389, -0.01157300, 0.04793006, -0.00851428,
        -0.00994407, -0.01350610, 0.04574149, -0.00442463,
        -0.00723813, -0.01610945, 0.04315705, -0.00054240,
        -0.00615762, -0.01800439, 0.04018005, 0.00309628,
        -0.00452335, -0.01961645, 0.03689702, 0.00646144,
        -0.00308805, -0.02080066, 0.03336996, 0.00952892,
        -0.00167556, -0.02160911, 0.02966901, 0.01228054,
        -0.00038565, -0.02204777, 0.02585923, 0.01470397,
        0.00077585, -0.02214873, 0.02200236, 0.01679246,
        0.00178585, -0.02194562, 0.01815440, 0.01854437,
        0.00263667, -0.02147808, 0.01436497, 0.01996270,
        0.00332554, -0.02078780, 0.01067657, 0.02105447,
        0.00385635, -0.01991718, 0.00712439, 0.02183008,
        0.00423774, -0.01890759, 0.00373629, 0.02230268,
        0.00448207, -0.01779815, 0.00053311, 0.02248756,
        0.00460428, -0.01662475, -0.00247085, 0.02240152,
        0.00462082, -0.01541935, -0.00526723, 0.02206236,
        0.00454878, -0.01420962, -0.00785284, 0.02148835,
        0.00440514, -0.01301875, -0.01022897, 0.02069787,
        0.00420625, -0.01186549, -0.01240053, 0.01970901,
        0.00396738, -0.01076430, -0.01437535, 0.01853929,
        0.00370251, -0.00972564, -0.01616343, 0.01720547,
        0.00342411, -0.00875633, -0.01777629, 0.01572334,
        0.00314318, -0.00785990, -0.01922633, 0.01410766,
        0.00286919, -0.00703701, -0.02052633, 0.01237208,
        0.00261022, -0.00628586, -0.02168897, 0.01052913,
        0.00237306, -0.00560251, -0.02272643, 0.00859028,
        0.00216331, -0.00498124, -0.02365003, 0.00656593,
        0.00198552, -0.00441489, -0.02447000, 0.00446558,
        0.00184335, -0.00389508, -0.02519518, 0.00229787,
        0.00173962, -0.00341249, -0.02583289, 0.00007074,
        0.00167642, -0.00295706, -0.02638874, -0.00220844,
        0.00165520, -0.00251818, -0.02686654, -0.00453269,
        0.00167672, -0.00208485, -0.02726823, -0.00689532,
        0.00174114, -0.00164588, -0.02759380, -0.00928968,
        0.00184791, -0.00119005, -0.02784128, -0.01170907,
        0.00199573, -0.00070627, -0.02800677, -0.01414652,
        0.00218246, -0.00018379, -0.02808444, -0.01659462,
        0.00240500, 0.00038758, -0.02806665, -0.01904540,
        0.00265916, 0.00101721, -0.02794406, -0.02149013,
        0.00293949, 0.00171337, -0.02770577, -0.02391919,
        0.00323917, 0.00248293, -0.02733958, -0.02632198,
        0.00354986, 0.00333094, -0.02683222, -0.02868675,
        0.00386166, 0.00426029, -0.02616976, -0.03100057,
        0.00416301, 0.00527127, -0.02533798, -0.03324930,
        0.00444082, 0.00636115, -0.02432286, -0.03541755,
        0.00468054, 0.00752375, -0.02311117, -0.03748873,
        0.00486650, 0.00874911, -0.02169109, -0.03944515,
        0.00498230, 0.01002316, -0.02005290, -0.04126819,
        0.00501137, 0.01132752, -0.01818971, -0.04293850,
        0.00493771, 0.01263950, -0.01609819, -0.04443625,
        0.00474687, 0.01393219, -0.01377937, -0.04574155,
        0.00442635, 0.01517491, -0.01123930, -0.04683477,
        0.00396885, 0.01633373, -0.00848965, -0.04769708,
        0.00336568, 0.01737277, -0.00554820, -0.04831091,
        0.00263355, 0.01825391, -0.00243919, -0.04866055,
        0.00172910, 0.01894337, 0.00080689, -0.04873265,
        0.00083346, 0.01939605, 0.00415263, -0.04851682,
        -0.00056680, 0.01961425, 0.00755692, -0.04800617,
        -0.00077582, 0.01946586, 0.01096731, -0.04719771,
        -0.00476426, 0.01926275, 0.01434701, -0.04609291,
        0.00162323, 0.01801547, 0.01760015, -0.04469737,
        -0.01724931, 0.01844043, 0.02078153, -0.04302319,
        0.00831948, 0.01392457, 0.02360591, -0.04108069,
        0.07672125, 0.01610261, 0.02622628, -0.03890912,
        0.00156066, 0.03618818, 0.03078951, -0.03644683,
        -0.01904254, 0.03659676, 0.03714119, -0.03348331,
        -0.01287426, 0.03161144, 0.04309348, -0.02997608,
};

static const aerodyn_coef_model_t aerodyn_dCY_dbeta_coef_model = {
        3.01824954, -0.47964431, -0.19015821, -0.41954436,
        1.39872592, 0.31053157, -0.20491608, -0.43778565,
        -0.61082814, 0.67671716, -0.11876238, -0.45237356,
        0.28716478, 0.51680272, -0.01460812, -0.45798998,
        -0.17460661, 0.59198229, 0.08215163, -0.45513826,
        -0.12187732, 0.54627038, 0.18148291, -0.44357701,
        -0.24672978, 0.51436298, 0.27404063, -0.42366055,
        -0.30643398, 0.44976927, 0.35817704, -0.39599286,
        -0.37487312, 0.36954504, 0.42967570, -0.36151448,
        -0.42367602, 0.27140349, 0.48560902, -0.32145308,
        -0.45724666, 0.16048537, 0.52329843, -0.27729041,
        -0.47086298, 0.04077847, 0.54086201, -0.23070571,
        -0.46377396, -0.08249317, 0.53722172, -0.18350898,
        -0.43544749, -0.20390891, 0.51222842, -0.13756397,
        -0.38679638, -0.31790879, 0.46669124, -0.09470585,
        -0.31982661, -0.41917185, 0.40236882, -0.05665742,
        -0.23764715, -0.50290226, 0.32190267, -0.02494884,
        -0.14427375, -0.56511814, 0.22870031, -0.00084529,
        -0.04440538, -0.60288892, 0.12677246, 0.01471308,
        0.05686036, -0.61451422, 0.02053400, 0.02115529,
        0.15433805, -0.59962821, -0.08541992, 0.01830521,
        0.24307230, -0.55922261, -0.18654873, 0.00638706,
        0.31863940, -0.49558643, -0.27859818, -0.01398957,
        0.37741027, -0.41216683, -0.35781459, -0.04186420,
        0.41675669, -0.31336105, -0.42112885, -0.07597743,
        0.43518791, -0.20425440, -0.46629932, -0.11483727,
        0.43240879, -0.09032248, -0.49200600, -0.15679583,
        0.40929874, 0.02288188, -0.49789130, -0.20013193,
        0.36781565, 0.13003604, -0.48454669, -0.24313488,
        0.31083490, 0.22632995, -0.45344790, -0.28418484,
        0.24193714, 0.30770634, -0.40684444, -0.32182546,
        0.16516332, 0.37104533, -0.34761218, -0.35482523,
        0.08475435, 0.41428499, -0.27907918, -0.38222468,
        0.00489534, 0.43647362, -0.20483649, -0.40336765,
        -0.07051904, 0.43775522, -0.12854563, -0.41791581,
        -0.13808383, 0.41929338, -0.05375403, -0.42584670,
        -0.19503668, 0.38314312, 0.01627177, -0.42743629,
        -0.23935860, 0.33208263, 0.07868699, -0.42322813,
        -0.26982078, 0.26941870, 0.13117788, -0.41399151,
        -0.28597985, 0.19877979, 0.17203591, -0.40067166,
        -0.28812741, 0.12391044, 0.20019594, -0.38433495,
        -0.27720320, 0.04847886, 0.21523974, -0.36611241,
        -0.25468105, -0.02409277, 0.21736783, -0.34714423,
        -0.22243941, -0.09076811, 0.20734433, -0.32852804,
        -0.18262602, -0.14900261, 0.18642039, -0.31127290,
        -0.13752611, -0.19681399, 0.15624219, -0.29626074,
        -0.08944163, -0.23281825, 0.11874971, -0.28421626,
        -0.04058595, -0.25623401, 0.07607185, -0.27568584,
        0.00700151, -0.26685939, 0.03042334, -0.27102563,
        0.05152087, -0.26502640, -0.01599245, -0.27039829,
        0.09145718, -0.25153826, -0.06107122, -0.27377794,
        0.12560703, -0.22759483, -0.10288348, -0.28096221,
        0.15309008, -0.19471099, -0.13973661, -0.29159024,
        0.17334538, -0.15463210, -0.17022255, -0.30516563,
        0.18612113, -0.10925039, -0.19325064, -0.32108274,
        0.19144589, -0.06052399, -0.20806625, -0.33865534,
        0.18962653, -0.01040357, -0.21425584, -0.35714623,
        0.18113241, 0.03924054, -0.21173934, -0.37579679,
        0.16684807, 0.08666089, -0.20075237, -0.39385532,
        0.14706643, 0.13034161, -0.18181533, -0.41060343,
        0.12469104, 0.16884351, -0.15570650, -0.42537947,
        0.09451816, 0.20148755, -0.12338902, -0.43759874,
        0.07775977, 0.22623235, -0.08606342, -0.44676923,
        0.01274599, 0.24658981, -0.04480190, -0.45250515,
        0.08898911, 0.24992670, -0.00147266, -0.45452849,
        -0.22487282, 0.27322400, 0.04418085, -0.45269457,
        0.42068919, 0.21435243, 0.08672992, -0.44690778,
        -0.62552830, 0.32448860, 0.13375267, -0.43742721,
        -3.14117974, 0.16072568, 0.17609561, -0.42369967,
        -0.19974359, -0.66163325, 0.13238317, -0.40919597,
        0.56790339, -0.71392600, 0.01234298, -0.40281473,
        0.33353624, -0.56524924, -0.09928612, -0.40679705,
};


static double eval_aerodyn_coef(double alpha, const aerodyn_alpha_list_t *list, const aerodyn_coef_model_t *model) {
    double C, interp_diff;
    int i, piece_num;
    Eigen::Matrix<double, 1, AERODYN_ORDER + 1> para;
    Eigen::Matrix<double, 1, AERODYN_ORDER + 1> x;

    /* locate the piece of coefficient in the piecewise model */
    piece_num = floor((alpha - *(*list)) / AERODYN_BREAK_INTERVAL);

    /* evaluate the model */
    interp_diff = alpha - *(*list + piece_num);

    x(AERODYN_ORDER) = 1.0f;

    for (i = AERODYN_ORDER - 1; i >= 0; i--) {
        x(i) = x(i + 1) * interp_diff;
    }

    memcpy(para.data(), *(*model + piece_num), (AERODYN_ORDER + 1) * sizeof(double));

    C = para.dot(x);

    return C;
}

static double eval_aerodyn_coef_D(double alpha, const aerodyn_alpha_list_t *list, const aerodyn_coef_model_t *model,
                                  const int d_order) {
    double dC, interp_diff;
    int i, piece_num, vec_size;
    Eigen::VectorXd para, x;

    vec_size = AERODYN_ORDER + 1 - d_order;
    para.resize(vec_size);
    x.resize(vec_size);

    /* locate the piece of coefficient in the piecewise model */
    piece_num = floor((alpha - *(*list)) / AERODYN_BREAK_INTERVAL);

    /* evaluate the model */
    interp_diff = alpha - *(*list + piece_num);

    switch (d_order) {
        case 1:
            x(AERODYN_ORDER - 1) = 1.0f;
            for (int i = AERODYN_ORDER - 2; i >= 0; i--) {
                x(i) = (double) (AERODYN_ORDER - i) / (double) (AERODYN_ORDER - i - 1) * x(i + 1) * interp_diff;
            }
            memcpy(para.data(), *(*model + piece_num), (AERODYN_ORDER) * sizeof(double));
            dC = para.dot(x);
            break;

        case 2:
            x(AERODYN_ORDER - 2) = 2.0f;
            for (int i = AERODYN_ORDER - 3; i >= 0; i--) {
                x(i) = (double) (AERODYN_ORDER - i) / (double) (AERODYN_ORDER - i - 2) * x(i + 1) * interp_diff;
            }
            memcpy(para.data(), *(*model + piece_num), (AERODYN_ORDER - 1) * sizeof(double));
            dC = para.dot(x);
            break;

        default:
            break;
    }
    return dC;
}

template<class T>
static T tailsitter_F_calc(T h, T gamma, T alpha) {
    T sin_alpha = sin(alpha);
    T cos_alpha = cos(alpha);
    T CL = eval_aerodyn_coef(alpha, &aerodyn_alpha_list, &aerodyn_CL_coef_model);
    T CD = eval_aerodyn_coef(alpha, &aerodyn_alpha_list, &aerodyn_CD_coef_model);
    T f = CD * sin_alpha + CL * cos_alpha;
    T F = h * sin(gamma - alpha) - f;
    return F;
}


template<class T>
struct F_dF_functor { // Functor also returning 1st derivative.
    F_dF_functor(T const &h, T const &gamma) : m_h(h), m_gamma(gamma) {
        ;
    }

    std::pair<T, T> operator()(T const &alpha) {
        T sin_alpha = sin(alpha);
        T cos_alpha = cos(alpha);
        T CL = eval_aerodyn_coef(alpha, &aerodyn_alpha_list, &aerodyn_CL_coef_model);
        T CD = eval_aerodyn_coef(alpha, &aerodyn_alpha_list, &aerodyn_CD_coef_model);
        T dCL = eval_aerodyn_coef_D(alpha, &aerodyn_alpha_list, &aerodyn_CL_coef_model, 1);
        T dCD = eval_aerodyn_coef_D(alpha, &aerodyn_alpha_list, &aerodyn_CD_coef_model, 1);
        T f = CD * sin_alpha + CL * cos_alpha;
        T F = m_h * sin(m_gamma - alpha) - f;
        T df = dCD * sin_alpha + CD * cos_alpha + dCL * cos_alpha - CL * sin_alpha;
        T dF = -m_h * cos(m_gamma - alpha) - df;
        return std::make_pair(F, dF);   // 'return' both F and dF.
    }

private:
    T m_h;
    T m_gamma;                          // Store value 
};

template<class T>
static T solve_alpha_func(T h, T gamma, T dir, T guess) {
    // return cube root of x using 1st derivative and Newton_Raphson.
    using namespace boost::math::tools;
// print_line;
    T min, max, F_0, F_upd, sub_result, abs_F_0, abs_F_upd, stepout, stepout_last, stepout_diff, convergent_thr;
    int count = 0;
    T stepout_dir[2], stepout_result[2], stepout_F[2], stepout_abs_F[2];
    sub_result = guess;
    stepout = ALPHA_SOLVER_STEPOUT;
    stepout_dir[0] = -1;
    stepout_dir[1] = 1;
    stepout_diff = ALPHA_SOLVER_STEPOUT;

    if (abs(h) < 1.0e+3) {
        convergent_thr = ALPHA_SOLVER_EPSILON;
    } else {
        convergent_thr = 10 * ALPHA_SOLVER_EPSILON;
    }

    if (abs(h) > INF && abs(gamma) < EPSILON) {
        return 0;
    }

    if (abs(h) > INF) {
        guess = dir * 0.25 * PI;
    }

    min = guess - ALPHA_SOLVER_SMALL_BOUND;
    max = guess + ALPHA_SOLVER_SMALL_BOUND;
    min = min > -PI ? min : -PI;
    max = max<
    PI ? max : PI;

    guess = guess < max ? (guess) : max;
    guess = guess > min ? (guess) : min;

    F_0 = tailsitter_F_calc(h, gamma, guess);
    abs_F_0 = abs(F_0);
// print_line;
    const int digits = std::numeric_limits<T>::digits;  // Maximum possible binary digits accuracy for type T.
    int get_digits = static_cast<int>(digits * 0.6);    // Accuracy doubles with each step, so stop when we have
    // just over half the digits correct.
    const std::uintmax_t maxit = 20;
    std::uintmax_t it = maxit;
    T result = newton_raphson_iterate(F_dF_functor<T>(h, gamma), guess, min, max, get_digits, it);

    F_upd = tailsitter_F_calc(h, gamma, result);
    abs_F_upd = abs(F_upd);

// print_line;
// std::cout << "h = " << h << ", gamma = " << gamma << ", dir = " << dir << ", guess = " << guess << ", result = " << result << ", F = " << F_upd << std::endl;

    if (abs_F_upd < convergent_thr ||
        ((abs_F_upd / abs_F_0) < convergent_thr)) // && abs(result) < ALPHA_SOLVER_EPSILON))
    {
        return result;
    }

    while (true) {
        for (int j = 0; j < 2; j++) {
            stepout_result[j] = result + stepout_dir[j] * stepout;
            min = (stepout_result[j] - stepout_diff);
            max = (stepout_result[j] + stepout_diff);
            min = min > -PI ? min : -PI;
            max = max<
            PI ? max : PI;
            stepout_result[j] = stepout_result[j] < max ? stepout_result[j] : max;
            stepout_result[j] = stepout_result[j] > min ? stepout_result[j] : min;

            if (abs(stepout_result[j]) >= PI) {
                continue;
            }
// print_line;
// std::cout << "result[j] = " << stepout_result[j]  << std::endl;
            stepout_result[j] = newton_raphson_iterate(F_dF_functor<T>(h, gamma), stepout_result[j], min, max,
                                                       get_digits, it);
// print_line;
// std::cout << "result[j] = " << stepout_result[j]  << std::endl;
            stepout_F[j] = tailsitter_F_calc(h, gamma, stepout_result[j]);
            stepout_abs_F[j] = abs(stepout_F[j]);

            if (stepout_abs_F[j] < convergent_thr || stepout_abs_F[j] / abs_F_0 < convergent_thr) {
// print_line;
// std::cout << "result[j] = " << stepout_result[j] << ", F[j] = " << stepout_F[j] << std::endl;
                return stepout_result[j];
            }
        }

        stepout_last = stepout;
        stepout *= ALPHA_SOLVER_STEPOUT_RATIO;
        stepout_diff *= ALPHA_SOLVER_STEPOUT_RATIO;
        count++;

        if (count > ALPHA_SOLVER_MAX_ITER) {
//            fmt::print(fg(fmt::color::red), " -- [WARNING] Maximum Iteriation. h = {}, gamma = {}, dir = {}, guess = {}, result = {}, residual = {} \n", h, gamma, dir, guess, sub_result, F_upd);
            if (abs_F_upd < abs_F_0) {
                return result;
            } else {
                return guess;
            }
        }
    }
}

template<class T>
static T sign_(T x) {
    return (x > 0) ? 1 : ((x < 0) ? -1 : 0);
}

static void Vector2SkewSym(Eigen::Matrix3d &skew, Eigen::Vector3d &v) {
    skew << 0.0, -v(2), v(1), v(2), 0.0, -v(0), -v(1), v(0), 0.0;
}

enum diffFlatOrder_t {
    ORD_2ND = 2,
    ORD_3RD = 3,
    ORD_4TH = 4,
};

enum singular_type {
    NONSINGULAR = 0,
    AIRSPEED_SINGLAR = 1,
    GAMMA_SINGLAR = 2,
};

struct aircraft_state_t {
    diffFlatOrder_t diffFlatOrder;
    singular_type singular;
    bool yaw_plan_en;
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Vector3d a;
    Eigen::Vector3d j;
    Eigen::Vector3d s;
    Eigen::Vector3d windspeed;
    Eigen::Vector3d va;
    Eigen::Vector3d x_b;
    Eigen::Vector3d y_b;
    Eigen::Vector3d z_b;
    Eigen::Vector3d z_f;
    Eigen::Vector3d dz_f;
    Eigen::Vector3d omega;
    Eigen::Vector3d domega;
    Eigen::Vector3d va_b;
    Eigen::Vector3d a_minus_g;
    Eigen::Vector3d c;
    Eigen::Vector3d pc_palpha;
    Eigen::Vector3d pc_pbeta;
    Eigen::Vector3d ppc_ppalpha;
    Eigen::Vector3d ppc_pbetapalpha;
    Eigen::Vector3d v_cross_amg;
    Eigen::Vector3d zf_cross_amg;
    Eigen::Vector3d euler;
    Eigen::Vector4d H;
    Eigen::Vector4d dThrAccRate;

    double alpha;
    double gamma;
    double aT;
    double daT;
    double ddaT;
    double dir;
    double va_norm;
    double va_2norm;
    double yaw;
    double dyaw;
    double amg_norm;
    double zf_cross_amg_norm;
    double h;
    double CL;
    double CD;
    double dCL;
    double dCD;
    double ddCL;
    double ddCD;
    double sin_alpha;
    double cos_alpha;
    double dalpha;
    double dgamma;
    double amg_2norm;
    double sin_gamma;
    double cos_gamma;

    Eigen::Quaterniond q;
    Eigen::Matrix3d R;
    Eigen::Matrix3d paa_pvb;
    Eigen::Matrix4d N;
    Eigen::Matrix4d N_inv;
    Eigen::Matrix<double, 3, 24> gradVelCC;
    Eigen::Matrix<double, 3, 24> gradAccCC;
    Eigen::Matrix<double, 3, 24> gradJerkCC;
    Eigen::Matrix<double, 3, 24> gradVCrossAmgCC;
    Eigen::Matrix<double, 4, 24> graddThrAccRateCC;
    Eigen::Matrix<double, 1, 24> gradThrAccCC;
    Eigen::Matrix<double, 8, 3> gradThrAccC;
    Eigen::Matrix<double, 8, 3> gradRateC_x;
    Eigen::Matrix<double, 8, 3> gradRateC_y;
    Eigen::Matrix<double, 8, 3> gradRateC_z;
    Eigen::Matrix<double, 8, 3> gradGammaC;
};

class TAILSITTER {
    // private:
public:
    const aerodyn_alpha_list_t *const alpha_list = &aerodyn_alpha_list;
    const aerodyn_coef_model_t *const CL_model = &aerodyn_CL_coef_model;
    const aerodyn_coef_model_t *const CD_model = &aerodyn_CD_coef_model;
    const aerodyn_coef_model_t *const CY_model = &aerodyn_CY_coef_model;
    const aerodyn_coef_model_t *const dCY_dbeta_model = &aerodyn_dCY_dbeta_coef_model;
    double m;                  // mass, unit: kg
    double S;                  // suqare unit: m^2
    double rho;                // air density, unit: kg/m^3
    double rhoS_2m;

public:

    aircraft_state_t aircraft_state;
    bool init_en;

    inline void aerodyn_init(double yaw_init) {
        init_en = true;
        m = TAILSITTER_MASS;
        S = TAILSITTER_SQUARE;
        rho = TAILSITTER_AIR_DENSITY;
        rhoS_2m = 0.5 * rho * S / m;

        aircraft_state.yaw = yaw_init;
        aircraft_state.y_b = Eigen::AngleAxisd(aircraft_state.yaw, Eigen::Vector3d::UnitZ()) * Eigen::Vector3d::UnitY();
        aircraft_state.z_f = Eigen::AngleAxisd(aircraft_state.yaw, Eigen::Vector3d::UnitZ()) * Eigen::Vector3d::UnitX();
        aircraft_state.alpha = 0.0;
    }

    inline void
    diff_flat_calc(aircraft_state_t &state, Eigen::Vector3d &vel, Eigen::Vector3d &acc, Eigen::Vector3d &jerk,
                   Eigen::Vector3d &snap) {
        double amg_dot_v;
        Eigen::Vector3d g_vec, yb_tmp, a_aero, e1, e2, e3;
        Eigen::Matrix3d E1, E2, aa_skewsym, vb_skewsym;
        e1 = Eigen::Vector3d::UnitX();
        e2 = Eigen::Vector3d::UnitY();
        e3 = Eigen::Vector3d::UnitZ();
        E1 << 0, 0, 0, 0, 0, -1, 0, 1, 0;
        E2 << 0, 0, 1, 0, 0, 0, -1, 0, 0;
        g_vec << 0, 0, GRAVITY;

        state.v = vel;
        state.a = acc;
        state.j = jerk;
        state.s = snap;
        state.va = state.v;// - state.windspeed;
        state.va_norm = state.va.norm();
        state.a_minus_g = state.a - g_vec;
        state.amg_norm = state.a_minus_g.norm();

        if (state.singular != NONSINGULAR) {
            aircraft_state.alpha = 0.0;
        }

        if (state.va_norm < VEL_MIN) {
            state.singular = AIRSPEED_SINGLAR;
            // if (state.yaw_plan_en == true)
            // {
            state.z_f = Eigen::AngleAxisd(state.yaw, e3) * Eigen::Vector3d::UnitX();
            state.dz_f = state.dyaw * e3.cross(state.z_f);
            // }
            // else
            // {
            //     state.dz_f.setZero();
            // }

            state.x_b = state.a_minus_g / state.amg_norm;
            state.aT = state.amg_norm;
            state.zf_cross_amg = state.z_f.cross(state.a_minus_g);
            state.zf_cross_amg_norm = state.zf_cross_amg.norm();
            state.y_b = state.zf_cross_amg / state.zf_cross_amg_norm;
            state.z_b = state.x_b.cross(state.y_b);
            state.R << state.x_b, state.y_b, state.z_b;
            state.H(0) = state.z_b.dot(state.z_f.cross(state.j) + state.dz_f.cross(state.a_minus_g));
            state.H.segment<3>(1) = state.j;
            state.N(0, 0) = 0.0;
            state.N.block<1, 3>(0, 1) = state.zf_cross_amg_norm * e1.transpose();
            state.N.block<3, 1>(1, 0) = state.x_b;
            state.N.block<3, 3>(1, 1) = -state.aT * state.R * E1;
        } else {
            state.v_cross_amg = state.va.cross(state.a_minus_g);
            state.dir = sign_(state.v_cross_amg.dot(state.y_b));
            state.va_2norm = state.va_norm * state.va_norm;
            amg_dot_v = state.a_minus_g.dot(state.va) / (state.amg_norm * state.va_norm);
            state.gamma = state.dir * acos(amg_dot_v);
            state.h = state.amg_norm / (rhoS_2m * state.va_2norm);
// print_line;
// std::cout << "h = " << state.h << ", gamma = " << state.gamma << ", dir = " << state.dir << ", guess = " << state.alpha << std::endl;
// std::cout << "vel = " << state.va.transpose() << ", acc = " << state.a.transpose() << ", jerk = " << state.j.transpose() << std::endl;
            state.alpha = solve_alpha_func(state.h, state.gamma, state.dir, state.alpha);
// std::cout << "result = " << state.alpha << std::endl;
            state.CL = eval_aerodyn_coef(state.alpha, alpha_list, CL_model);
            state.CD = eval_aerodyn_coef(state.alpha, alpha_list, CD_model);
            state.dCL = eval_aerodyn_coef_D(state.alpha, alpha_list, CL_model, 1);
            state.dCD = eval_aerodyn_coef_D(state.alpha, alpha_list, CD_model, 1);
            state.sin_alpha = sin(state.alpha);
            state.cos_alpha = cos(state.alpha);
            state.c(0) = -state.CD * state.cos_alpha + state.CL * state.sin_alpha;
            state.c(1) = 0.0;
            state.c(2) = -state.CD * state.sin_alpha - state.CL * state.cos_alpha;
            state.pc_palpha(0) = (-state.dCD + state.CL) * state.cos_alpha + (state.CD + state.dCL) * state.sin_alpha;
            state.pc_palpha(1) = 0.0;
            state.pc_palpha(2) = (-state.dCD + state.CL) * state.sin_alpha - (state.CD + state.dCL) * state.cos_alpha;
            state.pc_pbeta.setZero();
            state.pc_pbeta(1) = eval_aerodyn_coef(state.alpha, alpha_list, dCY_dbeta_model);
            a_aero = rhoS_2m * state.va_2norm * state.c;
            Vector2SkewSym(aa_skewsym, a_aero);
            state.aT = state.amg_norm * cos(state.gamma - state.alpha) - a_aero(0);

            if (abs(state.gamma) < GAMMA_MIN) {
                state.singular = GAMMA_SINGLAR;
                state.zf_cross_amg = state.z_f.cross(state.a_minus_g);
                state.zf_cross_amg_norm = state.zf_cross_amg.norm();
                state.y_b = state.zf_cross_amg / state.zf_cross_amg_norm;
                state.x_b = Eigen::AngleAxisd(state.alpha, state.y_b) * state.va / state.va_norm;
                state.z_b = state.x_b.cross(state.y_b);
                state.R << state.x_b, state.y_b, state.z_b;
                state.va_b = state.R.transpose() * state.va;
                Vector2SkewSym(vb_skewsym, state.va_b);
                state.paa_pvb = rhoS_2m * (2.0 * state.c * state.va_b.transpose() +
                                           state.pc_palpha * state.va_b.transpose() * E2 +
                                           state.va_norm * state.pc_pbeta * e2.transpose());
                state.H(0) = state.z_b.dot(state.z_f.cross(state.j));
                state.H.segment<3>(1) = state.j - state.R * state.paa_pvb * state.R.transpose() * state.a;;
                state.N(0, 0) = 0.0;
                state.N.block<1, 3>(0, 1) = state.zf_cross_amg_norm * e1.transpose();
                state.N.block<3, 1>(1, 0) = state.x_b;
                state.N.block<3, 3>(1, 1) = state.R * (-state.aT * E1 - aa_skewsym + state.paa_pvb * vb_skewsym);
            } else {
                state.singular = NONSINGULAR;
                state.y_b = state.dir * state.v_cross_amg.normalized();
                state.x_b = Eigen::AngleAxisd(state.alpha, state.y_b) * state.va / state.va_norm;
                state.z_b = state.x_b.cross(state.y_b);
                state.R << state.x_b, state.y_b, state.z_b;
                state.va_b = state.R.transpose() * state.va;
                Vector2SkewSym(vb_skewsym, state.va_b);
                state.paa_pvb = rhoS_2m * (2.0 * state.c * state.va_b.transpose() +
                                           state.pc_palpha * state.va_b.transpose() * E2 +
                                           state.va_norm * state.pc_pbeta * e2.transpose());
                state.H(0) = state.y_b.dot(state.a);
                state.H.segment<3>(1) = state.j - state.R * state.paa_pvb * state.R.transpose() * state.a;
                state.N(0, 0) = 0.0;
                state.N.block<1, 3>(0, 1) = state.va_b.transpose() * E2;
                state.N.block<3, 1>(1, 0) = state.x_b;
                state.N.block<3, 3>(1, 1) = state.R * (-state.aT * E1 - aa_skewsym + state.paa_pvb * vb_skewsym);
                state.z_f = state.z_b;
            }
        }

        state.N_inv = state.N.inverse();
        state.dThrAccRate = state.N_inv * state.H;
        // state.dThrAccRate = state.N.partialPivLu().solve(state.H);
        if (Eigen::isnan(state.dThrAccRate.array()).any()) {
            std::cout << "N = " << std::endl;
            std::cout << state.N << std::endl;
            std::cout << "H = " << state.H.transpose() << std::endl;
        }
        state.daT = state.dThrAccRate(0);
        state.omega << state.dThrAccRate(1), state.dThrAccRate(2), state.dThrAccRate(3);
        state.q = state.R;
        state.euler = state.R.eulerAngles(2, 0, 1);
        state.yaw = state.euler(0);


        if (state.diffFlatOrder == ORD_4TH) {
            Eigen::Matrix3d I_33, tmp_mat33, tmp_mat33_2, dR, dpaa_pvb_dt, RTN2_24;
            Eigen::Vector3d e_j, dvb_dt, daa_dt;

            vector<Eigen::Matrix3d> E_skewsym;
            Eigen::Vector4d dH_dt, ddThrAccdRate, tmp_vec_4;
            Eigen::Matrix4d dN_dt;
            Eigen::Matrix3d w_hat, zf_hat;

            I_33.setIdentity();
            Vector2SkewSym(zf_hat, state.z_f);
            Vector2SkewSym(w_hat, state.omega);
            for (int j = 0; j < 3; j++) {
                e_j = I_33.col(j);
                Vector2SkewSym(tmp_mat33, e_j);
                E_skewsym.push_back(tmp_mat33);
            }
            state.amg_2norm = state.amg_norm * state.amg_norm;

            if (state.singular == AIRSPEED_SINGLAR) {
                dH_dt(0) = state.z_b.dot(state.z_f.cross(state.s) + state.dz_f.cross(state.j)) +
                           state.z_f.cross(state.j).dot((state.R * w_hat.col(2)));
                dH_dt.segment<3>(1) = state.s;
                dN_dt(0, 0) = 0.0;
                dN_dt.block<1, 3>(0, 1) =
                        -state.a_minus_g.transpose() * zf_hat * (zf_hat * state.j + state.dz_f.cross(state.a_minus_g)) *
                        e1.transpose() / state.zf_cross_amg_norm;
                dN_dt.block<3, 1>(1, 0) = state.R * w_hat.col(0);
                dN_dt.block<3, 3>(1, 1) = -(state.daT * state.R + state.aT * state.R * w_hat) * E1;
            } else {
                state.sin_gamma = sin(state.gamma);
                state.cos_gamma = cos(state.gamma);

                dR = state.R * w_hat;
                dvb_dt = -w_hat * state.va_b + state.R.transpose() * state.a;
                state.dalpha = (dvb_dt(2) * state.va_b(0) - state.va_b(2) * dvb_dt(0)) / state.va_2norm;
                state.dgamma = state.dir / abs(state.sin_gamma) *
                               ((state.cos_gamma / state.amg_2norm * state.a_minus_g -
                                 state.va / state.va_norm / state.amg_norm).dot(state.j)
                                + (state.cos_gamma / state.va_2norm * state.va -
                                   state.a_minus_g / state.va_norm / state.amg_norm).dot(state.a));

                /* dpaa_pvb_dt */
                state.ddCD = eval_aerodyn_coef_D(state.alpha, alpha_list, CD_model, 2);
                state.ddCL = eval_aerodyn_coef_D(state.alpha, alpha_list, CL_model, 2);
                state.ppc_ppalpha(0) = (-state.ddCD + 2.0 * state.dCL + state.CD) * state.cos_alpha +
                                       (2.0 * state.dCD - state.CL + state.ddCL) * state.sin_alpha;
                state.ppc_ppalpha(1) = 0.0;
                state.ppc_ppalpha(2) = (-state.ddCD + 2.0 * state.dCL + state.CD) * state.sin_alpha +
                                       (-2.0 * state.dCD + state.CL - state.ddCL) * state.cos_alpha;
                state.ppc_pbetapalpha.setZero();
                state.ppc_pbetapalpha(1) = eval_aerodyn_coef_D(state.alpha, alpha_list, dCY_dbeta_model, 1);

                dpaa_pvb_dt = rhoS_2m * (2.0 * (state.pc_palpha * state.va_b.transpose() * state.dalpha +
                                                state.c * dvb_dt.transpose())
                                         + (state.ppc_ppalpha * state.va_b.transpose() * state.dalpha +
                                            state.pc_palpha * dvb_dt.transpose()) * E_skewsym[1]
                                         + (state.va.dot(state.a) / state.va_norm * state.pc_pbeta +
                                            state.va_norm * state.dalpha * state.ppc_pbetapalpha) * e2.transpose());
                daa_dt = rhoS_2m *
                         (2.0 * state.va.dot(state.a) * state.c + state.va_2norm * state.pc_palpha * state.dalpha);

                if (state.singular == GAMMA_SINGLAR) {
                    dH_dt(0) = state.z_b.dot(state.z_f.cross(state.s)) +
                               state.z_f.cross(state.j).dot((state.R * w_hat.col(2)));
                    dN_dt.block<1, 3>(0, 1) =
                            -state.a_minus_g.transpose() * zf_hat * (zf_hat * state.j) * e1.transpose() /
                            state.zf_cross_amg_norm;
                } else {
                    dH_dt(0) = dR.col(1).dot(state.a) + state.y_b.dot(state.j);
                    dN_dt.block<1, 3>(0, 1) = dvb_dt.transpose() * E_skewsym[1];
                }

                dH_dt.block<3, 1>(1, 0) = state.s - state.R *
                                                    ((w_hat * state.paa_pvb + dpaa_pvb_dt - state.paa_pvb * w_hat) *
                                                     (state.R.transpose() * state.a)
                                                     + state.paa_pvb * (state.R.transpose() * state.j));
                dN_dt(0, 0) = 0.0;
                dN_dt.block<3, 1>(1, 0) = dR.col(0);
                Vector2SkewSym(tmp_mat33, daa_dt);
                Vector2SkewSym(tmp_mat33_2, dvb_dt);
                RTN2_24 = state.R.transpose() * state.N.block<3, 3>(1, 1);
                dN_dt.block<3, 3>(1, 1) = dR * RTN2_24 + state.R * (-state.daT * E_skewsym[0] - tmp_mat33 +
                                                                    dpaa_pvb_dt * vb_skewsym +
                                                                    state.paa_pvb * tmp_mat33_2);
            }
            ddThrAccdRate = -state.N_inv * dN_dt * state.N_inv * state.H + state.N_inv * dH_dt;
            state.ddaT = ddThrAccdRate(0);
            state.domega << ddThrAccdRate(1), ddThrAccdRate(2), ddThrAccdRate(3);
        }
    }

    inline void gradCC_to_gradC(Eigen::Matrix<double, 8, 3> &gradC, const Eigen::Matrix<double, 1, 24> &gradCC) {
        Eigen::Matrix<double, 3, 8> gradC_T;
        for (int j = 0; j < 3; j++) {
            gradC_T.row(j) = gradCC.block<1, 8>(0, 8 * j);

        }
        gradC = gradC_T.transpose();
    }

    /* derive in Jacobian form, then transpose to Hessian form */
    /* CC = [c(:,1); c(:,2); c(:,3)] */
    inline void gradient_calc(aircraft_state_t &state) {
        double v_cross_amg_norm, v_cross_amg_2norm, sin_gamma_minus_alpha, cos_gamma_minus_alpha;
        double tmp_scalar_1, tmp_scalar_2;
        Eigen::Vector3d e_j, e1, e2, tmp_vec_1, tmp_vec_2, RTa, RTN2_24_ej;
        Eigen::Matrix3d I_33, zf_skewsym, xb_skewsym, yb_skewsym, va_skewsym, vb_skewsym, amg_skewsym, tmp_mat33, tmp_mat33_2, exp_ybAlpha, RTN2_24;
        Eigen::Matrix<double, 1, 24> p_gamma, p_h, p_alpha, p_w_alpha, tmp_mat1n;
        Eigen::Matrix<double, 3, 24> p_xb, p_yb, p_zb, p_vb, tmp_mat3n, tmp_mat3n_2, tmp_mat3n_3, tmp_mat3n_4, tmp_mat3n_5, p_N2_j;
        Eigen::Matrix<double, 4, 24> p_H, p_N1T, p_N_u;
        vector<Eigen::Matrix3d> E_skewsym;
        vector<Eigen::Matrix<double, 3, 24>> p_N2;

        e1 = Eigen::Vector3d::UnitX();
        e2 = Eigen::Vector3d::UnitY();
        I_33.setIdentity();
        for (int j = 0; j < 3; j++) {
            e_j = I_33.col(j);
            Vector2SkewSym(tmp_mat33, e_j);
            E_skewsym.push_back(tmp_mat33);
        }
        Vector2SkewSym(xb_skewsym, state.x_b);
        Vector2SkewSym(yb_skewsym, state.y_b);
        Vector2SkewSym(va_skewsym, state.va);
        Vector2SkewSym(amg_skewsym, state.a_minus_g);

        if (state.singular == AIRSPEED_SINGLAR) {
            state.gradThrAccCC = state.a_minus_g.transpose() / state.amg_norm * state.gradAccCC;
            Vector2SkewSym(zf_skewsym, state.z_f);
            p_xb = (state.amg_2norm * I_33 - state.a_minus_g * state.a_minus_g.transpose()) /
                   (state.amg_2norm * state.amg_norm) * state.gradAccCC;
            p_yb = (pow(state.zf_cross_amg_norm, 2) * I_33 - state.zf_cross_amg * state.zf_cross_amg.transpose()) /
                   pow(state.zf_cross_amg_norm, 3) * zf_skewsym * state.gradAccCC;
            p_zb = xb_skewsym * p_yb - yb_skewsym * p_xb;
            p_H.block<1, 24>(0, 0) =
                    (yb_skewsym * state.j).transpose() * p_zb + state.z_b.transpose() * zf_skewsym * state.gradJerkCC;
            p_H.block<3, 24>(1, 0) = state.gradJerkCC;
            p_N1T.setZero();
            p_N1T.block<3, 24>(1, 0) =
                    e1 / state.zf_cross_amg_norm * state.zf_cross_amg.transpose() * zf_skewsym * state.gradAccCC;
            /* partial N2 wrt. CC */
            p_N2.push_back(p_xb);
            for (int j = 0; j < 3; j++) {
                tmp_vec_1 = E_skewsym[j] * e1;
                p_N2_j = state.aT * (tmp_vec_1(0) * p_xb + tmp_vec_1(1) * p_yb + tmp_vec_1(2) * p_zb) -
                         state.R * tmp_vec_1 * state.gradThrAccCC;
                p_N2.push_back(p_N2_j);
            }
            p_N_u.row(0) = state.dThrAccRate.transpose() * p_N1T;
            p_N_u.block<3, 24>(1, 0) =
                    state.dThrAccRate(0) * p_N2[0] + state.dThrAccRate(1) * p_N2[1] + state.dThrAccRate(2) * p_N2[2] +
                    state.dThrAccRate(3) * p_N2[3];
// std::cout << "p_N_u = " << std::endl;
// std::cout << p_N_u << std::endl;
        } else {
            sin_gamma_minus_alpha = sin(state.gamma - state.alpha);
            cos_gamma_minus_alpha = cos(state.gamma - state.alpha);
            /* partial gamma wrt. CC */
            tmp_scalar_1 = state.dir / (state.amg_2norm * state.va_2norm * abs(state.sin_gamma));
            tmp_vec_1 = state.va_2norm * state.cos_gamma * state.a_minus_g - state.amg_norm * state.va_norm * state.v;
            tmp_vec_2 = state.amg_2norm * state.cos_gamma * state.v - state.amg_norm * state.va_norm * state.a_minus_g;
            p_gamma =
                    tmp_scalar_1 * (tmp_vec_1.transpose() * state.gradAccCC + tmp_vec_2.transpose() * state.gradVelCC);
            /* partial alpha wrt. CC */
            tmp_vec_1 = 1.0 / rhoS_2m / state.va_2norm / state.amg_norm * state.a_minus_g;
            tmp_vec_2 = 1.0 / rhoS_2m * 2.0 / (state.va_2norm * state.va_2norm) * state.amg_norm * state.v;
            p_h = tmp_vec_1.transpose() * state.gradAccCC - tmp_vec_2.transpose() * state.gradVelCC;
            tmp_scalar_1 = sin_gamma_minus_alpha / (state.h * cos_gamma_minus_alpha - state.pc_palpha(2));
            tmp_scalar_2 = state.h * cos_gamma_minus_alpha / (state.h * cos_gamma_minus_alpha - state.pc_palpha(2));
            p_alpha = tmp_scalar_1 * p_h + tmp_scalar_2 * p_gamma;
// std::cout << "p_alpha = " << std::endl;
// std::cout << p_alpha << std::endl;
            /* partial y_b wrt. CC */
            v_cross_amg_norm = state.v_cross_amg.norm();
            v_cross_amg_2norm = v_cross_amg_norm * v_cross_amg_norm;
            p_yb = state.dir / (v_cross_amg_norm * v_cross_amg_2norm) *
                   (v_cross_amg_2norm * I_33 - state.v_cross_amg * state.v_cross_amg.transpose())
                   * (-amg_skewsym * state.gradVelCC + va_skewsym * state.gradAccCC);
// std::cout << "p_yb = " << std::endl;
// std::cout << p_yb << std::endl;
            /* partial x_b wrt. CC */
            exp_ybAlpha = Eigen::AngleAxisd(state.alpha, state.y_b);
            tmp_vec_1 = state.va / state.va_norm;
            tmp_vec_2 = state.y_b.cross(tmp_vec_1);
            Vector2SkewSym(tmp_mat33, tmp_vec_2);
            tmp_mat33_2 = -state.sin_alpha / state.va_norm * va_skewsym -
                          (tmp_mat33 + yb_skewsym * va_skewsym / state.va_norm) * (1.0 - state.cos_alpha);
            p_xb = yb_skewsym * exp_ybAlpha * tmp_vec_1 * p_alpha + tmp_mat33_2 * p_yb +
                   exp_ybAlpha * (state.va_2norm * I_33 - state.va * state.va.transpose())
                   / (state.va_2norm * state.va_norm) * state.gradVelCC;
// std::cout << "p_xb = " << std::endl;
// std::cout << p_xb << std::endl;
            /* partial z_b wrt. CC */
            p_zb = xb_skewsym * p_yb - yb_skewsym * p_xb;
// std::cout << "p_zb = " << std::endl;
// std::cout << p_zb << std::endl;
            /* partial aT wrt. CC */
            state.gradThrAccCC = state.a_minus_g.transpose() / state.amg_norm * cos_gamma_minus_alpha * state.gradAccCC
                                 - state.amg_norm * sin_gamma_minus_alpha * (p_gamma - p_alpha) -
                                 e1.transpose() * rhoS_2m
                                 * (state.va_2norm * state.pc_palpha * p_alpha +
                                    2.0 * state.c * state.va.transpose() * state.gradVelCC);
// std::cout << "p_aT = " << std::endl;
// std::cout << state.gradThrAccCC << std::endl;
            /* partial vb wrt. CC */
            tmp_mat3n.row(0) = state.va.transpose() * p_xb;
            tmp_mat3n.row(1) = state.va.transpose() * p_yb;
            tmp_mat3n.row(2) = state.va.transpose() * p_zb;
            p_vb = tmp_mat3n + state.R.transpose() * state.gradVelCC;
            /* partial H1 wrt. CC */
            p_H.block<1, 24>(0, 0) = state.y_b.transpose() * state.gradAccCC + state.a.transpose() * p_yb;
            /* partial H2 wrt. CC */
            RTa = state.R.transpose() * state.a;
            tmp_vec_1 = state.paa_pvb * RTa;
            tmp_mat3n = tmp_vec_1(0) * p_xb + tmp_vec_1(1) * p_yb + tmp_vec_1(2) * p_zb;
            tmp_mat3n_2 = rhoS_2m * ((2.0 * state.pc_palpha * state.va_b.transpose() +
                                      state.ppc_ppalpha * state.va_b.transpose() * E_skewsym[1]
                                      + state.va_norm * state.ppc_pbetapalpha * e2.transpose()) * RTa * p_alpha +
                                     (2.0 * state.c * RTa.transpose()
                                      - state.pc_palpha * RTa.transpose() * E_skewsym[1] +
                                      state.pc_pbeta / state.va_norm * e2.transpose() * RTa * state.va_b.transpose())
                                     * p_vb);
            tmp_mat3n_3.row(0) = state.a.transpose() * p_xb;
            tmp_mat3n_3.row(1) = state.a.transpose() * p_yb;
            tmp_mat3n_3.row(2) = state.a.transpose() * p_zb;
            tmp_mat3n_4 = state.paa_pvb * state.R.transpose() * state.gradAccCC;
            p_H.block<3, 24>(1, 0) =
                    state.gradJerkCC - tmp_mat3n - state.R * (tmp_mat3n_2 + state.paa_pvb * tmp_mat3n_3 + tmp_mat3n_4);
// std::cout << "p_H = " << std::endl;
// std::cout << p_H << std::endl;
            /* partial N1^T wrt. CC */
            p_N1T.row(0).setZero();
            p_N1T.block<3, 24>(1, 0) = -E_skewsym[1] * p_vb;
            /* partial N2 wrt. CC */
            p_N2.push_back(p_xb);
            RTN2_24 = state.R.transpose() * state.N.block<3, 3>(1, 1);
            Vector2SkewSym(vb_skewsym, state.va_b);
            for (int j = 0; j < 3; j++) {
                RTN2_24_ej = RTN2_24.col(j);
                tmp_mat3n = RTN2_24_ej(0) * p_xb + RTN2_24_ej(1) * p_yb + RTN2_24_ej(2) * p_zb; // p_N22_1
                tmp_mat3n_2 = -E_skewsym[0].col(j) * state.gradThrAccCC;
                tmp_mat3n_3 = E_skewsym[j] * state.paa_pvb * p_vb;
                tmp_mat3n_4 = rhoS_2m * ((2.0 * state.pc_palpha * state.va_b.transpose() +
                                          state.ppc_ppalpha * state.va_b.transpose() * E_skewsym[1]
                                          + state.va_norm * state.ppc_pbetapalpha * e2.transpose()) *
                                         vb_skewsym.col(j) * p_alpha + (2.0 * state.c * vb_skewsym.col(j).transpose()
                                                                        - state.pc_palpha *
                                                                          vb_skewsym.col(j).transpose() * E_skewsym[1] +
                                                                        state.pc_pbeta / state.va_norm *
                                                                        e2.transpose() * vb_skewsym.col(j) *
                                                                        state.va_b.transpose())
                                                                       * p_vb);
                tmp_mat3n_5 = -state.paa_pvb * E_skewsym[j] * p_vb;
                p_N2_j = tmp_mat3n + state.R * (tmp_mat3n_2 + tmp_mat3n_3 + tmp_mat3n_4 + tmp_mat3n_5);
                p_N2.push_back(p_N2_j);
            }
            p_N_u.row(0) = state.dThrAccRate.transpose() * p_N1T;
            p_N_u.block<3, 24>(1, 0) =
                    state.dThrAccRate(0) * p_N2[0] + state.dThrAccRate(1) * p_N2[1] + state.dThrAccRate(2) * p_N2[2] +
                    state.dThrAccRate(3) * p_N2[3];
// std::cout << "p_N_u = " << std::endl;
// std::cout << p_N_u << std::endl;
        }

        /* partial N2 wrt. CC */
        state.graddThrAccRateCC = state.N.inverse() * (p_H - p_N_u);
// std::cout << "p_ddaTdomega = " << std::endl;
// std::cout << state.graddThrAccRateCC << std::endl;
        /* tranform to partial u wrt. C */
        gradCC_to_gradC(state.gradThrAccC, state.gradThrAccCC);
        gradCC_to_gradC(state.gradRateC_x, state.graddThrAccRateCC.row(1));
        gradCC_to_gradC(state.gradRateC_y, state.graddThrAccRateCC.row(2));
        gradCC_to_gradC(state.gradRateC_z, state.graddThrAccRateCC.row(3));
// std::cout << "gradThrAccC = " << std::endl;
// std::cout << state.gradThrAccC << std::endl;
    }
};


#endif