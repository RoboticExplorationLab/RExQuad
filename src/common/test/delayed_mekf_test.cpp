extern "C" {
#include "common/delayed_mekf.h"

#include <slap/slap.h>

#include "common/linear_algebra.h"
}

#include <fmt/core.h>
#include <fmt/ostream.h>
#include <gtest/gtest.h>

#include "filter_data.h"

namespace rexquad {

TEST(DelayedMEKFTests, ConstructAndFree) {
  rexquad_DelayedMEKF filter = rexquad_NewDelayedMEKF(10);
  rexquad_FreeDelayedMEKF(&filter);
}

TEST(DelayedMEKFTests, Initialize) {
  int delay_comp = 10;
  const int n = 16;
  const int e = 15;
  const int m = 6;
  // const int n0 = 13;

  rexquad_DelayedMEKF filter = rexquad_NewDelayedMEKF(delay_comp);
  double Wf[36];
  double Vf[225];
  double x0[13] = {0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  double b0[6];
  double Pf0[225];
  rexquad_SetIdentity(m, Wf, 1e-4);
  rexquad_SetIdentity(e, Vf, 1e-6);
  (void)x0;
  rexquad_SetConstant(m, 1, b0, 0.1);
  rexquad_SetIdentity(e, Pf0, 1.0);
  rexquad_InitializeDelayedMEKF(&filter, delay_comp, x0, Wf, Vf, b0, Pf0);
  double xf[16] = {0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
  const double* xf_ = rexquad_GetFilterState(&filter);
  double err = rexquad_NormSquaredDifference(n, xf, xf_);
  EXPECT_LT(err, 1e-10);
  rexquad_FreeDelayedMEKF(&filter);
}

TEST(DelayedMEKFTests, StatePrediction) {
  int delay_comp = 10;
  const int n = 16;
  const int e = 15;
  // const int m = 6;
  // const int n0 = 13;

  rexquad_DelayedMEKF filter = rexquad_NewDelayedMEKF(delay_comp);
  // rexquad_InitializeDelayMEKFDefault(&filter);
  double x0_[13] = {0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  double b0_[6] = {0.2, 0.3, 0.3, 0.1, -0.2, 0.11};
  rexquad_InitializeDelayedMEKF(&filter, delay_comp, x0_, NULL, NULL, b0_, NULL);

  // clang-format off
  // Inputs
  double y_imu_[6] = {0.25, 0.24, 0.23, 0.33, 0.32, 0.31};
  // Expected output
  double xp_expected_[16] = {0.0, 0.0, 1.0, 0.9999954587809342, 0.0011499947775980744, 0.0025999881928304293, 0.0009999954587809343, 0.0010123167588830374, -0.0008287459054153141, -0.09879442491863569, 0.2, 0.3, 0.3, 0.1, -0.2, 0.11};
  double Pp_expected_[225] = {1.0002, 0.0, 0.0, 0.0, 0.0, 0.0, 0.00999984480140959, -1.994001889477839e-5, 5.202252750539394e-5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0002, 0.0, 0.0, 0.0, 0.0, 2.0059617808521256e-5, 0.009999953550421879, -2.2947791576683005e-5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0002, 0.0, 0.0, 0.0, -5.197652792318515e-5, 2.3051790632111586e-5, 0.009999838351468173, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0001249997398778, -7.474864160706882e-11, -2.8749478630746103e-11, -1.4721846266432272e-11, 0.19619188878601515, -0.00045223469548924777, 0.0, 0.0, 0.0, -0.004999954587912455, -4.999954587912456e-6, 1.2999881928572386e-5, 0.0, 0.0, 0.0, -7.474864160706882e-11, 1.000124999603943, -6.499881929108657e-11, -0.19619188881270844, 1.490109702433428e-11, -0.0010197288629407149, 0.0, 0.0, 0.0, 4.999954587912456e-6, -0.004999954587912456, -5.749947776099325e-6, 0.0, 0.0, 0.0, -2.8749478630746103e-11, -6.499881929108657e-11, 1.0001249997479402, 0.00045223468248150407, 0.0010197288682058537, -1.7925081211129431e-13, 0.0, 0.0, 0.0, -1.2999881928572385e-5, 5.749947776099325e-6, -0.004999954587912456, 0.00999984480140959, 2.0059617808521256e-5, -5.197652792318515e-5, -1.4721846266432272e-11, -0.19619188881270844, 0.00045223468248150407, 1.0386943761369167, 4.613087473088492e-7, 0.00020008750022163267, -0.00999984480140959, -2.0059617808521256e-5, 5.197652792318515e-5, 1.0094824741025841e-6, -0.000987944806755562, 7.15125821628608e-6, -1.994001889477839e-5, 0.009999953550421879, 2.3051790632111586e-5, 0.19619188878601515, 1.490109702433428e-11, 0.0010197288682058537, 4.613087473088492e-7, 1.0386952115724393, -8.874333015635337e-5, 1.994001889477839e-5, -0.009999953550421879, -2.3051790632111586e-5, 0.00098796159626089, 9.762937392713152e-7, 7.554443927708874e-6, 5.202252750539394e-5, -2.2947791576683005e-5, 0.009999838351468173, -0.00045223469548924777, -0.0010197288629407149, -1.7925081211129431e-13, 0.00020008750022163267, -8.874333015635337e-5, 1.0002012446750408, -5.202252750539394e-5, 2.2947791576683005e-5, -0.009999838351468173, -8.277260708343926e-6, -1.013136302977981e-5, 9.905660845478784e-9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.00999984480140959, 1.994001889477839e-5, -5.202252750539394e-5, 1.000001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -2.0059617808521256e-5, -0.009999953550421879, 2.2947791576683005e-5, 0.0, 1.000001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.197652792318515e-5, -2.3051790632111586e-5, -0.009999838351468173, 0.0, 0.0, 1.000001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.004999954587912455, 4.999954587912456e-6, -1.2999881928572385e-5, 1.0094824741025841e-6, 0.00098796159626089, -8.277260708343926e-6, 0.0, 0.0, 0.0, 1.000001, 0.0, 0.0, 0.0, 0.0, 0.0, -4.999954587912456e-6, -0.004999954587912456, 5.749947776099325e-6, -0.000987944806755562, 9.762937392713152e-7, -1.013136302977981e-5, 0.0, 0.0, 0.0, 0.0, 1.000001, 0.0, 0.0, 0.0, 0.0, 1.2999881928572386e-5, -5.749947776099325e-6, -0.004999954587912456, 7.15125821628608e-6, 7.554443927708874e-6, 9.905660845478784e-9, 0.0, 0.0, 0.0, 0.0, 0.0, 1.000001};
  // clang-format on
  Matrix xp_expected = slap_MatrixFromArray(n, 1, xp_expected_);
  Matrix Pp_expected = slap_MatrixFromArray(e, e, Pp_expected_);
  double h = 0.01;

  // Compute State prediction
  Matrix xd = slap_MatrixFromArray(n, 1, filter.xd);
  Matrix Pd = slap_MatrixFromArray(e, e, filter.Pd);
  Matrix xp = slap_MatrixFromArray(n, 1, filter.xp);
  Matrix Pp = slap_MatrixFromArray(e, e, filter.Pp);
  rexquad_StatePrediction(&filter, xp.data, Pp.data, xd.data, y_imu_, Pd.data, h);

  // Check predicted state
  double err = slap_MatrixNormedDifference(&xp, &xp_expected);
  EXPECT_LT(err, 1e-10);

  // Check predicted covariance
  double err_cov = slap_MatrixNormedDifference(&Pp, &Pp_expected);
  EXPECT_LT(err_cov, 1e-10);

  rexquad_FreeDelayedMEKF(&filter);
}

TEST(DelayedMEKFTests, MeasurementUpdate) {
  int delay_comp = 10;
  const int n = 16;
  const int e = 15;
  // const int m = 6;
  // const int n0 = 13;

  rexquad_DelayedMEKF filter = rexquad_NewDelayedMEKF(delay_comp);
  // rexquad_InitializeDelayMEKFDefault(&filter);
  double x0_[13] = {0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  rexquad_InitializeDelayedMEKF(&filter, delay_comp, x0_, NULL, NULL, NULL, NULL);

  // clang-format off
  // Define the inputs
  double xp_[16] = {0.0, 0.0, 1.0, 0.9999954587809342, 0.0011499947775980744, 0.0025999881928304293, 0.0009999954587809343, 0.0010123167588830374, -0.0008287459054153141, -0.09879442491863569, 0.2, 0.3, 0.3, 0.1, -0.2, 0.11}; double Pp_[225] = {1.0002, 0.0, 0.0, 0.0, 0.0, 0.0, 0.00999984480140959, -1.994001889477839e-5, 5.202252750539394e-5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0002, 0.0, 0.0, 0.0, 0.0, 2.0059617808521256e-5, 0.009999953550421879, -2.2947791576683005e-5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0002, 0.0, 0.0, 0.0, -5.197652792318515e-5, 2.3051790632111586e-5, 0.009999838351468173, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0001249997398778, -7.474864160706882e-11, -2.8749478630746103e-11, -1.4721846266432272e-11, 0.19619188878601515, -0.00045223469548924777, 0.0, 0.0, 0.0, -0.004999954587912455, -4.999954587912456e-6, 1.2999881928572386e-5, 0.0, 0.0, 0.0, -7.474864160706882e-11, 1.000124999603943, -6.499881929108657e-11, -0.19619188881270844, 1.490109702433428e-11, -0.0010197288629407149, 0.0, 0.0, 0.0, 4.999954587912456e-6, -0.004999954587912456, -5.749947776099325e-6, 0.0, 0.0, 0.0, -2.8749478630746103e-11, -6.499881929108657e-11, 1.0001249997479402, 0.00045223468248150407, 0.0010197288682058537, -1.7925081211129431e-13, 0.0, 0.0, 0.0, -1.2999881928572385e-5, 5.749947776099325e-6, -0.004999954587912456, 0.00999984480140959, 2.0059617808521256e-5, -5.197652792318515e-5, -1.4721846266432272e-11, -0.19619188881270844, 0.00045223468248150407, 1.0386943761369167, 4.613087473088492e-7, 0.00020008750022163267, -0.00999984480140959, -2.0059617808521256e-5, 5.197652792318515e-5, 1.0094824741025841e-6, -0.000987944806755562, 7.15125821628608e-6, -1.994001889477839e-5, 0.009999953550421879, 2.3051790632111586e-5, 0.19619188878601515, 1.490109702433428e-11, 0.0010197288682058537, 4.613087473088492e-7, 1.0386952115724393, -8.874333015635337e-5, 1.994001889477839e-5, -0.009999953550421879, -2.3051790632111586e-5, 0.00098796159626089, 9.762937392713152e-7, 7.554443927708874e-6, 5.202252750539394e-5, -2.2947791576683005e-5, 0.009999838351468173, -0.00045223469548924777, -0.0010197288629407149, -1.7925081211129431e-13, 0.00020008750022163267, -8.874333015635337e-5, 1.0002012446750408, -5.202252750539394e-5, 2.2947791576683005e-5, -0.009999838351468173, -8.277260708343926e-6, -1.013136302977981e-5, 9.905660845478784e-9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.00999984480140959, 1.994001889477839e-5, -5.202252750539394e-5, 1.000001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -2.0059617808521256e-5, -0.009999953550421879, 2.2947791576683005e-5, 0.0, 1.000001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.197652792318515e-5, -2.3051790632111586e-5, -0.009999838351468173, 0.0, 0.0, 1.000001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.004999954587912455, 4.999954587912456e-6, -1.2999881928572385e-5, 1.0094824741025841e-6, 0.00098796159626089, -8.277260708343926e-6, 0.0, 0.0, 0.0, 1.000001, 0.0, 0.0, 0.0, 0.0, 0.0, -4.999954587912456e-6, -0.004999954587912456, 5.749947776099325e-6, -0.000987944806755562, 9.762937392713152e-7, -1.013136302977981e-5, 0.0, 0.0, 0.0, 0.0, 1.000001, 0.0, 0.0, 0.0, 0.0, 1.2999881928572386e-5, -5.749947776099325e-6, -0.004999954587912456, 7.15125821628608e-6, 7.554443927708874e-6, 9.905660845478784e-9, 0.0, 0.0, 0.0, 0.0, 0.0, 1.000001};
  double y_mocap_[7] = {0.1, 0.2, 1.1, 0.9950371902099893, 0.0, 0.0, 0.09950371902099893};

  // Define the expected output
  double xn_expected_[16] = {0.09999000299910027, 0.19998000599820054, 1.0999900029991003, 0.9950381653658451, 1.1441404812104466e-7, 2.5867523923080486e-7, 0.09949396697937578, 0.002542951080394612, 0.0009953357470088337, -0.09779095783139653, 0.2, 0.3, 0.3, 0.10000574865431712, -0.19998700304241349, 0.10950516128817485};
  double Pn_expected_[225] = {9.999000299910027e-5, 0.0, 0.0, 0.0, 0.0, 0.0, 9.996845747685288e-7, -1.993403868317344e-9, 5.200692542776562e-9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9.999000299910027e-5, 0.0, 0.0, 0.0, 0.0, 2.0053601728002855e-9, 9.996954464082654e-7, -2.294090930389184e-9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9.999000299910027e-5, 0.0, 0.0, 0.0, -5.196093964129276e-9, 2.30448771689609e-9, 9.99683929967827e-7, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9.999000224949127e-5, -7.471501611827363e-19, -2.8736545749950137e-19, -2.9364019072249118e-15, 1.9614775559205865e-5, -4.5213296576576655e-8, 0.0, 0.0, 0.0, -4.998829852495951e-7, -4.998830225902406e-10, 1.299695747237844e-9, 0.0, 0.0, 0.0, -7.471501611827365e-19, 9.99900022494899e-5, -6.4969579735386515e-19, -1.9614775564534455e-5, 2.9622476526534224e-15, -1.019499475996192e-7, 0.0, 0.0, 0.0, 4.998829478759031e-10, -4.998829853175313e-7, -5.748654655025273e-10, 0.0, 0.0, 0.0, -2.8736545749950137e-19, -6.496957973538652e-19, 9.999000224949135e-5, 4.521329399346108e-8, 1.0194994867174627e-7, -2.5845750848220523e-17, 0.0, 0.0, 0.0, -1.299695775974129e-9, 5.748654005335378e-10, -4.998829852455657e-7, 9.996845747685283e-7, 2.0053601728002855e-9, -5.196093964129277e-9, -2.9364019072249118e-15, -1.9614775564534458e-5, 4.5213293993461075e-8, 1.000111602991289, 2.6441998984542766e-10, 6.997237583386142e-8, -0.00999984480140959, -2.0059617808521256e-5, 5.197652792318515e-5, 1.9960898728666624e-6, -0.001968677277244173, 8.28406303268039e-6, -1.993403868317344e-9, 9.996954464082656e-7, 2.30448771689609e-9, 1.9614775559205865e-5, 2.962247652653422e-15, 1.0194994867174629e-7, 2.6441998984538713e-10, 1.0001116032996407, -3.850960488971052e-8, 1.994001889477839e-5, -0.009999953550421879, -2.3051790632111586e-5, 0.00196870472011494, 1.9511616890572115e-6, 1.0101997400615336e-5, 5.200692542776561e-9, -2.2940909303891836e-9, 9.99683929967827e-7, -4.5213296576576655e-8, -1.019499475996192e-7, -2.5845750848220526e-17, 6.99723758339381e-8, -3.850960488971722e-8, 1.000100030582788, -5.202252750539394e-5, 2.2947791576683005e-5, -0.009999838351468173, -1.0532807553791914e-5, -1.5231074756456356e-5, 9.921265979434474e-9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.00999984480140959, 1.994001889477839e-5, -5.202252750539394e-5, 1.000001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -2.0059617808521256e-5, -0.009999953550421879, 2.2947791576683005e-5, 0.0, 1.000001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.197652792318515e-5, -2.3051790632111586e-5, -0.009999838351468173, 0.0, 0.0, 1.000001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -4.998829852495951e-7, 4.998829478759029e-10, -1.299695775974129e-9, 1.9960898728666624e-6, 0.00196870472011494, -1.0532807553791914e-5, 0.0, 0.0, 0.0, 0.9999760058837917, 7.472995969904504e-11, 2.8742292191910045e-11, 0.0, 0.0, 0.0, -4.998830225902406e-10, -4.998829853175312e-7, 5.748654005335378e-10, -0.001968677277244173, 1.951161689057212e-6, -1.5231074756456358e-5, 0.0, 0.0, 0.0, 7.472995969904707e-11, 0.9999760060196929, 6.498257365134119e-11, 0.0, 0.0, 0.0, 1.2996957472378444e-9, -5.748654655025274e-10, -4.998829852455657e-7, 8.28406303268039e-6, 1.0101997400615336e-5, 9.92126597943447e-9, 0.0, 0.0, 0.0, 2.8742292191931364e-11, 6.498257365133942e-11, 0.9999760058757315};
  // clang-format on
  Matrix xn_expected = slap_MatrixFromArray(n, 1, xn_expected_);
  Matrix Pn_expected = slap_MatrixFromArray(e, e, Pn_expected_);
  Matrix xn = slap_MatrixFromArray(n, 1, filter.xn);
  Matrix Pn = slap_MatrixFromArray(e, e, filter.Pn);

  // Process the MOCAP measurement
  rexquad_MeasurementUpdate(&filter, xn.data, Pn.data, xp_, Pp_, y_mocap_);
  double err_mean = slap_MatrixNormedDifference(&xn, &xn_expected);
  double err_cov = slap_MatrixNormedDifference(&Pn, &Pn_expected);
  slap_MatrixAddition(&Pn_expected, &Pn, &Pn_expected, -1.0);

  EXPECT_LT(err_mean, 1e-12);
  EXPECT_LT(err_cov, 1e-12);

  rexquad_FreeDelayedMEKF(&filter);
}

TEST(DelayedMEKFTests, SimTest) {
  // Initialize the filter
  int delay_comp = 5;
  rexquad_DelayedMEKF filter = rexquad_NewDelayedMEKF(delay_comp);
  const double* x0_ = xhist[0];
  double b0_[6] = {0,0,0, 0,0,0};
  rexquad_InitializeDelayedMEKF(&filter, delay_comp, x0_, NULL, NULL, b0_, NULL);
  double h = 0.01;

  // Get variables
  Matrix xhat = slap_MatrixFromArray(13, 1, filter.xhat);
  Matrix xhat_expected;
  double err;
  const double* y_imu_;
  const double* y_mocap_;
  int i;

  // First Update
  for (i = 0; i < 10; ++i) {
    y_imu_ = imuhist[i];
    if (i < MOCAP_DELAY) {
      y_mocap_ = nullptr;
    } else {
      y_mocap_ = mocaphist[i - MOCAP_DELAY];
    }
    rexquad_UpdateStateEstimate(&filter, y_imu_, y_mocap_, h);
    xhat_expected = slap_MatrixFromArray(13, 1, xhist[i+1]);
    err = slap_MatrixNormedDifference(&xhat, &xhat_expected);
    EXPECT_LT(err, 1e-12);
  }
  (void)err;

  rexquad_FreeDelayedMEKF(&filter);
}

//TEST(DelayedMEKFTests, SingleUpdateNoMocap) {
//  int delay_comp = 10;
//  rexquad_DelayedMEKF filter = rexquad_NewDelayedMEKF(delay_comp);
//  double h = 0.01;
//
//  // Initialize the filter
//  double x0_[13] = {0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//  double b0_[6] = {0.2, 0.3, 0.3, 0.1, -0.2, 0.11};
//  rexquad_InitializeDelayedMEKF(&filter, delay_comp, x0_, NULL, NULL, b0_, NULL);
//
//  // clang-format off
//   double y_imu_[6] = {0.25, 0.24, 0.23, 0.33, 0.32, 0.31};
//   double y_mocap_[7] = {0.1, 0.2, 1.1, 0.9950371902099893, 0.0, 0.0, 0.09950371902099893};
//   double xhat_expected_[13] = {0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.23, 0.52, 0.2};
//   double xd_expected_[16] = {0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.3, 0.3, 0.1, -0.2, 0.11};
//  // clang-format on
//  Matrix xhat_expected = slap_MatrixFromArray(13, 1, xhat_expected_);
//  Matrix xd_expected = slap_MatrixFromArray(16, 1, xd_expected_);
//
//  // Process a single IMU measurement
//  rexquad_UpdateStateEstimate(&filter, y_imu_, NULL, h);
//  const Matrix xhat =
//      slap_MatrixFromArray(13, 1, (double*)rexquad_GetStateEstimate(&filter));
//  const Matrix xd = slap_MatrixFromArray(16, 1, (double*)rexquad_GetDelayedState(&filter));
//  (void)y_mocap_;
//  rexquad_FreeDelayedMEKF(&filter);
//  double err = slap_MatrixNormedDifference(&xhat, &xhat_expected);
//  EXPECT_LT(err, 1e-10);
//  err = slap_MatrixNormedDifference(&xd, &xd_expected);
//  EXPECT_LT(err, 1e-10);
//
//  EXPECT_EQ(rexquad_VectorQueueSize(&filter.imuhist), 1);
//}
//
//TEST(DelayedMEKFTests, SingleUpdateWithMocap) {
//  int delay_comp = 10;
//  rexquad_DelayedMEKF filter = rexquad_NewDelayedMEKF(delay_comp);
//  double h = 0.01;
//
//  // Initialize the filter
//  double x0_[13] = {0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//  double b0_[6] = {0.2, 0.3, 0.3, 0.1, -0.2, 0.11};
//  rexquad_InitializeDelayedMEKF(&filter, delay_comp, x0_, NULL, NULL, b0_, NULL);
//
//  // clang-format off
//   double y_imu_[6] = {0.25, 0.24, 0.23, 0.33, 0.32, 0.31};
//   double y_mocap_[7] = {0.1, 0.2, 1.1, 0.9950371902099893, 0.0, 0.0,
//   0.09950371902099893}; double xhat_expected_[13] = {0.09999000299910027,
//   0.19998000599820054, 1.0999900029991003,
//   0.9950381653658451, 1.1441404812104466e-7, 2.5867523923080486e-7, 0.09949396697937578,
//   0.002542951080394612, 0.0009953357470088337, -0.09779095783139653, 0.2299942513456829,
//   0.5199870030424135, 0.20049483871182516};
//  // clang-format on
//  Matrix xhat_expected = slap_MatrixFromArray(13, 1, xhat_expected_);
//
//  // Process a single IMU measurement
//  rexquad_UpdateStateEstimate(&filter, y_imu_, y_mocap_, h);
//  const Matrix xhat =
//      slap_MatrixFromArray(13, 1, (double*)rexquad_GetStateEstimate(&filter));
//  (void)y_mocap_;
//  rexquad_FreeDelayedMEKF(&filter);
//  double err = slap_MatrixNormedDifference(&xhat, &xhat_expected);
//  EXPECT_LT(err, 1e-10);
//  EXPECT_EQ(rexquad_VectorQueueSize(&filter.imuhist), 1);
//}
//
//TEST(DelayedMEKFTests, DoubleUpdate) {
//  printf("\nDOUBLE UPDATE\n");
//  int delay_comp = 10;
//  rexquad_DelayedMEKF filter = rexquad_NewDelayedMEKF(delay_comp);
//  double h = 0.01;
//
//  // Initialize the filter
//  double x0_[13] = {0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//  double b0_[6] = {0.2, 0.3, 0.3, 0.1, -0.2, 0.11};
//  rexquad_InitializeDelayedMEKF(&filter, delay_comp, x0_, NULL, NULL, b0_, NULL);
//
//  // clang-format off
//  double y_imu_[6] = {0.25, 0.24, 0.23, 0.33, 0.32, 0.31};
//  double y_mocap_[7] = {0.1, 0.2, 1.1, 0.9950371902099893, 0.0, 0.0, 0.09950371902099893};
//  double xhat_expected_[13] = {0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.23, 0.52, 0.2};
//  double xd_expected_[16] = {0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.3, 0.3, 0.1, -0.2, 0.11};
//  // clang-format on
//  Matrix xhat_expected = slap_MatrixFromArray(13, 1, xhat_expected_);
//  Matrix xd_expected = slap_MatrixFromArray(16, 1, xd_expected_);
//
//  // Process a single IMU measurement
//  rexquad_UpdateStateEstimate(&filter, y_imu_, NULL, h);
//  const Matrix xhat =
//      slap_MatrixFromArray(13, 1, (double*)rexquad_GetStateEstimate(&filter));
//  const Matrix xd = slap_MatrixFromArray(16, 1, (double*)rexquad_GetDelayedState(&filter));
//  (void)y_mocap_;
//  rexquad_FreeDelayedMEKF(&filter);
//  double err = slap_MatrixNormedDifference(&xhat, &xhat_expected);
//  EXPECT_LT(err, 1e-10);
//  err = slap_MatrixNormedDifference(&xd, &xd_expected);
//  EXPECT_LT(err, 1e-10);
//
//  // Process another IMU Measurement with MOCAP
//  rexquad_UpdateStateEstimate(&filter, y_imu_, y_mocap_, h);
//  // slap_PrintRowVector(&xhat);
//  double xhat_expected2_[13] = {
//      0.09999000299910027,  0.19998000599820054,   1.0999900029991003,
//      0.9950381653658451,   1.1441404812104466e-7, 2.5867523923080486e-7,
//      0.09949396697937578,  0.002542951080394612,  0.0009953357470088337,
//      -0.09779095783139653, 0.2299942513456829,    0.5199870030424135,
//      0.20049483871182516};
//  xhat_expected = slap_MatrixFromArray(13, 1, xhat_expected2_);
//  err = slap_MatrixNormedDifference(&xhat, &xhat_expected);
//  EXPECT_LT(err, 1e-10);
//
//  EXPECT_EQ(rexquad_VectorQueueSize(&filter.imuhist), 2);
//}
//
//TEST(DelayedMEKFTests, DelayedMocap) {
//  puts("\n\nDELAYED MOCAP");
//  int delay_comp = 10;
//  rexquad_DelayedMEKF filter = rexquad_NewDelayedMEKF(delay_comp);
//  double h = 0.01;
//
//  // Initialize the filter
//  double x0_[13] = {0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//  double b0_[6] = {0.2, 0.3, 0.3, 0.1, -0.2, 0.11};
//  rexquad_InitializeDelayedMEKF(&filter, delay_comp, x0_, NULL, NULL, b0_, NULL);
//
//  // clang-format off
//  double y_imu_[6] = {0.25, 0.24, 0.23, 0.33, 0.32, 0.31};
//  double y_mocap_[7] = {0.1, 0.2, 1.1, 0.9950371902099893, 0.0, 0.0, 0.09950371902099893};
//  double xhat_expected_[13] = {0.10025356330416452, 0.20004525027071396, 1.0940925466947753, 0.9946982592300542, 0.0026568332501449975, 0.008104472616561605, 0.10248234894550586, 0.022169800111545734, -0.0036187957643460836, -0.39392081453497385, 0.2299942513456829, 0.5199870030424135, 0.20049483871182516};
////  double xd_expected_[16] = {0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.3, 0.3, 0.1, -0.2, 0.11};
//  // clang-format on
//  Matrix xhat_expected = slap_MatrixFromArray(13, 1, xhat_expected_);
//
//  // Process multiple IMU Measurements
//  rexquad_UpdateStateEstimate(&filter, y_imu_, NULL, h);
//  y_imu_[0] += 0.1;
//  rexquad_UpdateStateEstimate(&filter, y_imu_, NULL, h);
//  y_imu_[0] += 0.1;
//  rexquad_UpdateStateEstimate(&filter, y_imu_, NULL, h);
//  y_imu_[0] += 0.1;
//  rexquad_UpdateStateEstimate(&filter, y_imu_, NULL, h);
//
//  // Process another IMU Measurement with MOCAP
//  rexquad_UpdateStateEstimate(&filter, y_imu_, y_mocap_, h);
//
//  // Check output
//  const Matrix xhat =
//      slap_MatrixFromArray(13, 1, (double*)rexquad_GetStateEstimate(&filter));
//  double err = slap_MatrixNormedDifference(&xhat, &xhat_expected);
//  EXPECT_LT(err, 1e-2);  // TODO: figure out why this isn't zero
//
//  // EXPECT_EQ(rexquad_VectorQueueSize(&filter.imuhist), 2);
//}

}  // namespace rexquad