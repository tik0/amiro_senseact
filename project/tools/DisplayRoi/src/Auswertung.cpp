/*
 * Auswertung.cpp
 *
 * Code generation for function 'Auswertung'
 *
 * C source code generated on: Wed Jul 15 11:14:12 2015
 *
 */

/* Include files */
#include "Auswertung.h"
#include "Auswertung_emxutil.h"
#include "Auswertung_data.h"

#include <rsb/Factory.h>

/* Variable Definitions */
static emxArray_uint8_T *more_Data;
static boolean_T more_Data_not_empty;
static unsigned char running_variable;

/* Function Declarations */
static int b_compute_nones(const boolean_T x[1080000]);
static void b_eml_li_find(const boolean_T x[1080000], emxArray_int32_T *y);
static int compute_nones(const emxArray_boolean_T *x);
static void eml_li_find(const emxArray_boolean_T *x, emxArray_int32_T *y);
static double rt_roundd(double u);

/* Function Definitions */
static int b_compute_nones(const boolean_T x[1080000])
{
  int k;
  int i;
  k = 0;
  for (i = 0; i < 1080000; i++) {
    if (x[i]) {
      k++;
    }
  }

  return k;
}

static void b_eml_li_find(const boolean_T x[1080000], emxArray_int32_T *y)
{
  int j;
  int i;
  j = y->size[0];
  y->size[0] = b_compute_nones(x);
  emxEnsureCapacity((emxArray__common *)y, j, (int)sizeof(int));
  j = 0;
  for (i = 0; i < 1080000; i++) {
    if (x[i]) {
      y->data[j] = i + 1;
      j++;
    }
  }
}

static int compute_nones(const emxArray_boolean_T *x)
{
  int k;
  int i;
  k = 0;
  for (i = 0; i < 158700; i++) {
    if (x->data[i]) {
      k++;
    }
  }

  return k;
}

static void eml_li_find(const emxArray_boolean_T *x, emxArray_int32_T *y)
{
  int k;
  int i;
  k = compute_nones(x);
  i = y->size[0];
  y->size[0] = k;
  emxEnsureCapacity((emxArray__common *)y, i, (int)sizeof(int));
  k = 0;
  for (i = 0; i < 158700; i++) {
    if (x->data[i]) {
      y->data[k] = i + 1;
      k++;
    }
  }
}

static double rt_roundd(double u)
{
  double y;
  if (fabs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = floor(u + 0.5);
    } else if (u > -0.5) {
      y = 0.0;
    } else {
      y = ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

rsb::Factory& factory = rsb::getFactory();
rsb::patterns::RemoteServerPtr remoteServer
    = factory.createRemoteServer("/mapServer");

void Auswertung(double X, double Y, double Phi_k, double, double, double, double
                update_picture)
{
  int i0;
  unsigned char ImOut[158700];
  double teta;
  int t;
  int s;
  double i;
  double j;
  int inFloorJ;
  int inFloorI;
  int inCeilingJ;
  int inCeilingI;
  double deltaJ;
  int i1;
  unsigned char u0;
  unsigned char u1;
  unsigned char u2;
  emxArray_boolean_T *r0;
  emxArray_int32_T *r1;
  static unsigned char b_ImOut[1080000];
  static boolean_T bv0[1080000];
  emxArray_int32_T *r2;

  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  /* globale Variabeln */
  if (!more_Data_not_empty) {
    i0 = more_Data->size[0] * more_Data->size[1] * more_Data->size[2];
    more_Data->size[0] = 600;
    more_Data->size[1] = 600;
    more_Data->size[2] = 3;
    emxEnsureCapacity((emxArray__common *)more_Data, i0, (int)sizeof(unsigned
      char));
    for (i0 = 0; i0 < 1080000; i0++) {
      more_Data->data[i0] = 0;
    }

    more_Data_not_empty = TRUE;
  }

  i0 = (int)(running_variable + 1U);
  if ((unsigned int)i0 > 255U) {
    i0 = 255;
  }

  running_variable = (unsigned char)i0;
  if (running_variable == update_picture) {
    i0 = more_Data->size[0] * more_Data->size[1] * more_Data->size[2];
    more_Data->size[0] = 230;
    more_Data->size[1] = 230;
    more_Data->size[2] = 3;
    emxEnsureCapacity((emxArray__common *)more_Data, i0, (int)sizeof(unsigned
      char));
    more_Data_not_empty = TRUE;
    for (i0 = 0; i0 < 158700; i0++) {
      more_Data->data[i0] = 0;

      /*  more_Data=Imrotate5(Harvester,(Phi_k),230,230); */
      /*   VerResOut=230; */
      /*   HorResOut=230; */
      /* Input Image Size */
      /* evaluate output image size after crop */
      /* image alocation 'for speed */
      ImOut[i0] = 0;
    }

    /*    XFactor=XSizeIn/VerResOut; % scaling factor for x axis (row) in case res out smaller than res in */
    /*    YFactor=YSizeIn/HorResOut; % scaling factor for y axis (coloumn) in case res out smaller than res in  */
    /*  convert angle to Radians */
    teta = Phi_k / 180.0 * 3.1415926535897931;

    /*  operating zoom, rotation, resolution change.  */
    for (t = 0; t < 230; t++) {
      for (s = 0; s < 230; s++) {
        i = (((1.0 + (double)t) - 115.0) * cos(teta) + ((1.0 + (double)s) -
              115.0) * sin(teta)) + 115.0;

        /*  evaluate row index in original image */
        j = (-((1.0 + (double)t) - 115.0) * sin(teta) + ((1.0 + (double)s) -
              115.0) * cos(teta)) + 115.0;

        /*  evaluate coloumn index in original image */
        if ((i > 1.0) && (j > 1.0) && (i <= 229.0) && (j <= 229.0)) {
          /* check if evaluated [i,j] index exits outside of image */
          /* crop image */
          /*  after [i,j] indexes are evaluated as if ROI  was relocated to left corner (as if top of ROI indexes is [1,1], */
          /*  then  we move [i,j] to ROI by [Xstart,Ystat]. */
          i++;

          /* move row index */
          j++;

          /* move coloumn index */
          /*  Bilinear Interpolation Algorithm  */
          /*  round the values of i,j          */
          inFloorJ = (int)floor(j);
          inFloorI = (int)floor(i);
          inCeilingJ = (int)ceil(j) - 1;
          inCeilingI = (int)ceil(i) - 1;
          deltaJ = j - (double)inFloorJ;
          j = i - (double)inFloorI;

          /* weighted avarage between two bottom pixels */
          /* weighted avarage between two top pixels  */
          for (i0 = 0; i0 < 3; i0++) {
            i1 = (int)rt_roundd((1.0 - deltaJ) * (double)Harvester[((inFloorI +
              230 * (inFloorJ - 1)) + 52900 * i0) - 1]);
            if (i1 < 256) {
              if (i1 >= 0) {
                u0 = (unsigned char)i1;
              } else {
                u0 = 0;
              }
            } else {
              u0 = MAX_uint8_T;
            }

            i1 = (int)rt_roundd(deltaJ * (double)Harvester[((inFloorI + 230 *
              inCeilingJ) + 52900 * i0) - 1]);
            if (i1 < 256) {
              if (i1 >= 0) {
                u1 = (unsigned char)i1;
              } else {
                u1 = 0;
              }
            } else {
              u1 = MAX_uint8_T;
            }

            i1 = (int)((unsigned int)u0 + u1);
            if ((unsigned int)i1 > 255U) {
              i1 = 255;
            }

            i1 = (int)rt_roundd((1.0 - j) * (double)i1);
            if (i1 < 256) {
              if (i1 >= 0) {
                u0 = (unsigned char)i1;
              } else {
                u0 = 0;
              }
            } else {
              u0 = MAX_uint8_T;
            }

            i1 = (int)rt_roundd((1.0 - deltaJ) * (double)Harvester[(inCeilingI +
              230 * (inFloorJ - 1)) + 52900 * i0]);
            if (i1 < 256) {
              if (i1 >= 0) {
                u1 = (unsigned char)i1;
              } else {
                u1 = 0;
              }
            } else {
              u1 = MAX_uint8_T;
            }

            i1 = (int)rt_roundd(deltaJ * (double)Harvester[(inCeilingI + 230 *
              inCeilingJ) + 52900 * i0]);
            if (i1 < 256) {
              if (i1 >= 0) {
                u2 = (unsigned char)i1;
              } else {
                u2 = 0;
              }
            } else {
              u2 = MAX_uint8_T;
            }

            i1 = (int)((unsigned int)u1 + u2);
            if ((unsigned int)i1 > 255U) {
              i1 = 255;
            }

            i1 = (int)rt_roundd(j * (double)i1);
            if (i1 < 256) {
              if (i1 >= 0) {
                u1 = (unsigned char)i1;
              } else {
                u1 = 0;
              }
            } else {
              u1 = MAX_uint8_T;
            }

            i1 = (int)((unsigned int)u0 + u1);
            if ((unsigned int)i1 > 255U) {
              i1 = 255;
            }

            ImOut[(t + 230 * s) + 52900 * i0] = (unsigned char)i1;
          }

          /* weighted avarage between two values  avaraged values */
          /*          if I(t,s,:)<0 I(t,s,:)=0; end; */
          /*          if I(t,s,:)>255 I(t,s,:)=255; end; */
        }
      }
    }

    i0 = more_Data->size[0] * more_Data->size[1] * more_Data->size[2];
    more_Data->size[0] = 230;
    more_Data->size[1] = 230;
    more_Data->size[2] = 3;
    emxEnsureCapacity((emxArray__common *)more_Data, i0, (int)sizeof(unsigned
      char));
    for (i0 = 0; i0 < 158700; i0++) {
      more_Data->data[i0] = ImOut[i0];
    }

    emxInit_boolean_T(&r0, 3);
    more_Data_not_empty = TRUE;
    i0 = r0->size[0] * r0->size[1] * r0->size[2];
    r0->size[0] = more_Data->size[0];
    r0->size[1] = more_Data->size[1];
    r0->size[2] = 3;
    emxEnsureCapacity((emxArray__common *)r0, i0, (int)sizeof(boolean_T));
    t = more_Data->size[0] * more_Data->size[1] * more_Data->size[2];
    for (i0 = 0; i0 < t; i0++) {
      r0->data[i0] = (more_Data->data[i0] == 0);
    }

    emxInit_int32_T(&r1, 1);
    eml_li_find(r0, r1);
    t = r1->size[0];
    emxFree_boolean_T(&r0);
    for (i0 = 0; i0 < t; i0++) {
      more_Data->data[r1->data[i0] - 1] = MAX_uint8_T;
    }

    emxFree_int32_T(&r1);

    /* % Roi Bild anzeigen */
    memset(&Roi[0], 255, 1080000U * sizeof(unsigned char));
    if (185.0 + X > (415.0 + X) - 1.0) {
      i0 = 0;
    } else {
      i0 = (int)(185.0 + X) - 1;
    }

    if (185.0 + Y > (415.0 + Y) - 1.0) {
      i1 = 0;
    } else {
      i1 = (int)(185.0 + Y) - 1;
    }

    for (s = 0; s < 3; s++) {
      t = more_Data->size[1];
      for (inFloorJ = 0; inFloorJ < t; inFloorJ++) {
        inFloorI = more_Data->size[0];
        for (inCeilingJ = 0; inCeilingJ < inFloorI; inCeilingJ++) {
          Roi[((i0 + inCeilingJ) + 600 * (i1 + inFloorJ)) + 360000 * s] =
            more_Data->data[(inCeilingJ + more_Data->size[0] * inFloorJ) +
            more_Data->size[0] * more_Data->size[1] * s];
        }
      }
    }

    /*  Roi=Imrotate5(Roi,90,600,600); */
    /*   VerResOut=230; */
    /*   HorResOut=230; */
    /* Input Image Size */
    /* evaluate output image size after crop */
    /* image alocation 'for speed */
    memset(&b_ImOut[0], 0, 1080000U * sizeof(unsigned char));

    /*    XFactor=XSizeIn/VerResOut; % scaling factor for x axis (row) in case res out smaller than res in */
    /*    YFactor=YSizeIn/HorResOut; % scaling factor for y axis (coloumn) in case res out smaller than res in  */
    /*  convert angle to Radians */
    /*  operating zoom, rotation, resolution change.  */
    for (t = 0; t < 600; t++) {
      for (s = 0; s < 600; s++) {
        i = (((1.0 + (double)t) - 300.0) * 6.123233995736766E-17 + ((1.0 +
               (double)s) - 300.0)) + 300.0;

        /*  evaluate row index in original image */
        j = (-((1.0 + (double)t) - 300.0) + ((1.0 + (double)s) - 300.0) *
             6.123233995736766E-17) + 300.0;

        /*  evaluate coloumn index in original image */
        if ((i > 1.0) && (j > 1.0) && (i <= 599.0)) {
          /* check if evaluated [i,j] index exits outside of image */
          /* crop image */
          /*  after [i,j] indexes are evaluated as if ROI  was relocated to left corner (as if top of ROI indexes is [1,1], */
          /*  then  we move [i,j] to ROI by [Xstart,Ystat]. */
          i++;

          /* move row index */
          j++;

          /* move coloumn index */
          /*  Bilinear Interpolation Algorithm  */
          /*  round the values of i,j          */
          inFloorJ = (int)floor(j);
          inFloorI = (int)floor(i);
          inCeilingJ = (int)ceil(j) - 1;
          inCeilingI = (int)ceil(i) - 1;
          deltaJ = j - (double)inFloorJ;
          j = i - (double)inFloorI;

          /* weighted avarage between two bottom pixels */
          /* weighted avarage between two top pixels  */
          for (i0 = 0; i0 < 3; i0++) {
            i1 = (int)rt_roundd((1.0 - deltaJ) * (double)Roi[((inFloorI + 600 *
                                  (inFloorJ - 1)) + 360000 * i0) - 1]);
            if (i1 < 256) {
              if (i1 >= 0) {
                u0 = (unsigned char)i1;
              } else {
                u0 = 0;
              }
            } else {
              u0 = MAX_uint8_T;
            }

            i1 = (int)rt_roundd(deltaJ * (double)Roi[((inFloorI + 600 *
              inCeilingJ) + 360000 * i0) - 1]);
            if (i1 < 256) {
              if (i1 >= 0) {
                u1 = (unsigned char)i1;
              } else {
                u1 = 0;
              }
            } else {
              u1 = MAX_uint8_T;
            }

            i1 = (int)((unsigned int)u0 + u1);
            if ((unsigned int)i1 > 255U) {
              i1 = 255;
            }

            i1 = (int)rt_roundd((1.0 - j) * (double)i1);
            if (i1 < 256) {
              if (i1 >= 0) {
                u0 = (unsigned char)i1;
              } else {
                u0 = 0;
              }
            } else {
              u0 = MAX_uint8_T;
            }

            i1 = (int)rt_roundd((1.0 - deltaJ) * (double)Roi[(inCeilingI + 600 *
                                 (inFloorJ - 1)) + 360000 * i0]);
            if (i1 < 256) {
              if (i1 >= 0) {
                u1 = (unsigned char)i1;
              } else {
                u1 = 0;
              }
            } else {
              u1 = MAX_uint8_T;
            }

            i1 = (int)rt_roundd(deltaJ * (double)Roi[(inCeilingI + 600 *
              inCeilingJ) + 360000 * i0]);
            if (i1 < 256) {
              if (i1 >= 0) {
                u2 = (unsigned char)i1;
              } else {
                u2 = 0;
              }
            } else {
              u2 = MAX_uint8_T;
            }

            i1 = (int)((unsigned int)u1 + u2);
            if ((unsigned int)i1 > 255U) {
              i1 = 255;
            }

            i1 = (int)rt_roundd(j * (double)i1);
            if (i1 < 256) {
              if (i1 >= 0) {
                u1 = (unsigned char)i1;
              } else {
                u1 = 0;
              }
            } else {
              u1 = MAX_uint8_T;
            }

            i1 = (int)((unsigned int)u0 + u1);
            if ((unsigned int)i1 > 255U) {
              i1 = 255;
            }

            b_ImOut[(t + 600 * s) + 360000 * i0] = (unsigned char)i1;
          }

          /* weighted avarage between two values  avaraged values */
          /*          if I(t,s,:)<0 I(t,s,:)=0; end; */
          /*          if I(t,s,:)>255 I(t,s,:)=255; end; */
        }
      }
    }

    for (i0 = 0; i0 < 1080000; i0++) {
      Roi[i0] = b_ImOut[i0];
      bv0[i0] = (Roi[i0] == 0);
    }

    emxInit_int32_T(&r2, 1);
    b_eml_li_find(bv0, r2);
    t = r2->size[0];
    for (i0 = 0; i0 < t; i0++) {
      Roi[r2->data[i0] - 1] = MAX_uint8_T;
    }

    emxFree_int32_T(&r2);
    for (i0 = 0; i0 < 3; i0++) {
      for (i1 = 0; i1 < 351; i1++) {
        Roi[(i1 + 360000 * i0) + 74524] = 20;
        Roi[(i1 + 360000 * i0) + 284524] = 20;
      }

      for (i1 = 0; i1 < 351; i1++) {
        Roi[124 + (600 * (124 + i1) + 360000 * i0)] = 20;
        Roi[474 + (600 * (124 + i1) + 360000 * i0)] = 20;
      }
    }

    /*       hold on; */
    /*      imshow(Roi); */
    running_variable = 0;
    cv::Mat image(cv::Size(600, 600), CV_8UC1, Roi, cv::Mat::AUTO_STEP);
    cv::flip(image, image, 1);
    cv::Point2f srcCenter(image.cols/2.0F, image.rows/2.0F);
    cv::Mat rot = cv::getRotationMatrix2D(srcCenter, 90, 1);
    cv::Mat dst, dstColor;

    cv::warpAffine(image, dst, rot, image.size(), cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(255));


//    // Load a color image and put it under the gray scale combine harvester
//    cv::cvtColor(dst, dstColor, cv::COLOR_GRAY2BGR, 3);  // Convert to color image
//    cv::Mat source = cv::imread("/opt/vbShare/CLAAS/600x600Map.bmp");
//    const uchar grayScaleThreshold = 230;
//    for (int idy = 0; idy < dst.rows; ++idy) {
//      for (int idx = 0; idx < dst.cols; ++idx) {
//        if (dst.at<uchar>(idy, idx) > grayScaleThreshold) {
//          dstColor.at<cv::Vec3b>(idy, idx) = source.at<cv::Vec3b>(idy, idx);
//        }
//      }
//    }

    // Load a color image and put it under the gray scale combine harvester
     cv::cvtColor(dst, dstColor, cv::COLOR_GRAY2BGR, 3);  // Convert to color image
     // UNCOMMENT FOR MAPSERVER SUPPORT
//     // Request the map as an image
//     boost::shared_ptr<std::string> result;
//     try {
//       result = remoteServer->call<std::string>("compressedMapImageReq", 1 /*second*/);
//     } catch (...) {
//       std::cout << "Map server timeout occurred";
//       cv::namedWindow( "DisplayRoi");// Create a window for display.
//       cv::imshow( "DisplayRoi", dst );                   // Show our image inside it.
//       cv::waitKey(1);
//       return;
//     }
//     cv::Mat rawCompressedData = cv::Mat( 1, result->size(), CV_8UC1, &(*result)[0] );
//     cv::Mat source = cv::imdecode( rawCompressedData, CV_LOAD_IMAGE_COLOR );
// //    cv::Mat source = cv::imread("/opt/vbShare/CLAAS/600x600Map.bmp");
// 
//     const uchar grayScaleThreshold = 230;
//     for (int idy = 0; idy < dst.rows; ++idy) {
//       for (int idx = 0; idx < dst.cols; ++idx) {
//         if (dst.at<uchar>(idy, idx) > grayScaleThreshold) {
//           dstColor.at<cv::Vec3b>(idy, idx) = source.at<cv::Vec3b>(idy, idx);
//         }
//       }
//     }

//    // Test of copying the non interleaved data from Matlab to an interleaved
//    // Mat array
//    cv::Mat imageR(cv::Size(600, 600), CV_8UC1, &Roi[0]);
//    cv::Mat imageG(cv::Size(600, 600), CV_8UC1, &Roi[600*600-1]);
//    cv::Mat imageB(cv::Size(600, 600), CV_8UC1, &Roi[600*600*2-1]);
//    std::vector<cv::Mat> channels;
//    channels.push_back(imageB);
//    channels.push_back(imageG);
//    channels.push_back(imageR);
//    cv::Mat image(cv::Size(600, 600), CV_8UC3, Roi);
//    cv::merge(channels, image);
//
//    // Merge the two images
//    const int bgrStep = 3;
//    int idxC = 0;
//    for (int idx = 0; idx < dst.rows * dst.cols; ++idx) {
//      if (dst.data[idx] == 255) {
//        std::cout << "1 " << int(dst.data[idx]) << std::endl << std::flush;
//        dstColor.data[idxC] = source.data[idxC];
//        dstColor.data[idxC+1] = source.data[idxC+1];
//        dstColor.data[idxC+2] = source.data[idxC+2];
//      }
//      idxC+=bgrStep;
//    }
//

    cv::namedWindow( "DisplayRoi");// Create a window for display.
    cv::imshow( "DisplayRoi", dstColor );                   // Show our image inside it.
    cv::waitKey(1);

  }
  /*  */
}

void Auswertung_free()
{
  emxFree_uint8_T(&more_Data);
}

void Auswertung_init()
{
  running_variable = 1;
  emxInit_uint8_T(&more_Data, 3);
  more_Data_not_empty = FALSE;
}

/* End of code generation (Auswertung.cpp) */
