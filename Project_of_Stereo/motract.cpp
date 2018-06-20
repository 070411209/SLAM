/*
 * Copyright 2018 <copyright holder> <email>
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 */


#include <iostream>
#include <vector>
#include <list>
#include "motract.h"


using namespace cv;
using namespace std;



vector<cv::Mat> mvImagePyramid;

vector<cv::Point> pattern;

vector<int> 	mnFeaturesPerLevel;
vector<int> 	umax;
vector<float> 	mvScaleFactor;
vector<float> 	mvInvScaleFactor;    
vector<float> 	mvLevelSigma2;
vector<float> 	mvInvLevelSigma2;


enum {HARRIS_SCORE=0, FAST_SCORE=1 };

/**
 * 参数初始化 :
 * @param  mvInvScaleFactor mvInvLevelSigma2 umax  mnFeaturesPerLevel
 */

void ORBextractor(/*int nfeatures, float scaleFactor, int nlevels,int iniThFAST, int minThFAST*/)
{
                           
    mvScaleFactor.resize(nlevels);
    mvLevelSigma2.resize(nlevels);
    mvScaleFactor[0]=1.0f;
    mvLevelSigma2[0]=1.0f;
    for(int i=1; i<nlevels; i++)
    {
        mvScaleFactor[i]=mvScaleFactor[i-1]*scaleFactor;
        mvLevelSigma2[i]=mvScaleFactor[i]*mvScaleFactor[i];
    }

    mvInvScaleFactor.resize(nlevels);
    mvInvLevelSigma2.resize(nlevels);
    for(int i=0; i<nlevels; i++)
    {
        mvInvScaleFactor[i]=1.0f/mvScaleFactor[i];
        mvInvLevelSigma2[i]=1.0f/mvLevelSigma2[i];
    }

    mvImagePyramid.resize(nlevels);

    mnFeaturesPerLevel.resize(nlevels);
    float factor = 1.0f / scaleFactor;
    float nDesiredFeaturesPerScale = nfeatures*(1 - factor)/(1 - (float)pow((double)factor, (double)nlevels));

    int sumFeatures = 0;
    for( int level = 0; level < nlevels-1; level++ )
    {
        mnFeaturesPerLevel[level] = cvRound(nDesiredFeaturesPerScale);
        sumFeatures += mnFeaturesPerLevel[level];
        nDesiredFeaturesPerScale *= factor;
    }
    mnFeaturesPerLevel[nlevels-1] = std::max(nfeatures - sumFeatures, 0);

    //const int npoints = 512;
    const Point* pattern0 = (const Point*)bit_pattern_31_;
    std::copy(pattern0, pattern0 + npoints, std::back_inserter(pattern));

    //This is for orientation
    // pre-compute the end of a row in a circular patch
    umax.resize(HALF_PATCH_SIZE + 1);

    int v, v0, vmax = cvFloor(HALF_PATCH_SIZE * sqrt(2.f) / 2 + 1);
    int vmin = cvCeil(HALF_PATCH_SIZE * sqrt(2.f) / 2);
    const double hp2 = HALF_PATCH_SIZE*HALF_PATCH_SIZE;
    for (v = 0; v <= vmax; ++v)
        umax[v] = cvRound(sqrt(hp2 - v * v));

    // Make sure we are symmetric
    for (v = HALF_PATCH_SIZE, v0 = 0; v >= vmin; --v)
    {
        while (umax[v0] == umax[v0 + 1])
            ++v0;
        umax[v] = v0;
        ++v0;
    }
}

/*
void ORBextractor() {
	// Resizes the vector so that it contains n elements
    mvScaleFactor.resize(nlevels);
    mvLevelSigma2.resize(nlevels);
    mvScaleFactor[0]=1.0f;
    mvLevelSigma2[0]=1.0f;
    for(int i=1; i<nlevels; i++)
    {
		// scaleFactor=2.0f
        mvScaleFactor[i]=mvScaleFactor[i-1]*scaleFactor;
        mvLevelSigma2[i]=mvScaleFactor[i]*mvScaleFactor[i];
    }
	
	// 尺度因子
    mvInvScaleFactor.resize(nlevels);
	// 模糊因子
    mvInvLevelSigma2.resize(nlevels);
    for(int i=0; i<nlevels; i++)
    {
        mvInvScaleFactor[i]=1.0f/mvScaleFactor[i];
        mvInvLevelSigma2[i]=1.0f/mvLevelSigma2[i];
		
		cout << "mvInvScaleFactor["<< i << "]:" << mvInvScaleFactor[i] << "  mvInvLevelSigma2["<< i << "]:" << mvInvLevelSigma2[i] << endl;
    }
	// 图像金字塔层
    mvImagePyramid.resize(nlevels);
	// 特征点层
    mnFeaturesPerLevel.resize(nlevels);
    float factor = 1.0f / scaleFactor;
	// pow（x，y）求的是x的y次方
    float nDesiredFeaturesPerScale = nfeatures*(1 - factor)/(1 - (float)pow((double)factor, (double)nlevels));

	// 打印每层特征点缩放因子
	cout << "nDesiredFeaturesPerScale : " << nDesiredFeaturesPerScale << endl;
    int sumFeatures = 0;
    for( int level = 0; level < nlevels-1; level++ )
    {
        mnFeaturesPerLevel[level] = cvRound(nDesiredFeaturesPerScale);
        sumFeatures += mnFeaturesPerLevel[level];
        nDesiredFeaturesPerScale *= factor;
    }
    // 顶层特征点个数
    mnFeaturesPerLevel[nlevels-1] = max(nfeatures - sumFeatures, 0);
	// 打印每层特征点个数
	cout << "mnFeaturesPerLevel : " << mnFeaturesPerLevel[0] << " - " << mnFeaturesPerLevel[1] << " - " << mnFeaturesPerLevel[2] << endl;

	// ?? -2018-04-27
    const Point* pattern0 = (const Point*)bit_pattern_31_;
	
	// Copies the elements in the range [first,last) into the range beginning at result.
	// copy (InputIterator first, InputIterator last, OutputIterator result);
    std::copy(pattern0, pattern0 + npoints, std::back_inserter(pattern));

    // This is for orientation
    // pre-compute the end of a row in a circular patch
	// 计算定位圆形大小
    umax.resize(HALF_PATCH_SIZE + 1);
	cout << "umax size : "<< umax.size() << endl;
	//  cvFloor() 返回不大于参数的最大整数值
	// double sqrt (double x): Returns the square root of x.
    int v, v0, vmax = cvFloor(HALF_PATCH_SIZE * sqrt(2.f) / 2 + 1);
	// cvCeil 返回不小于参数的最小整数值
    int vmin = cvCeil(HALF_PATCH_SIZE * sqrt(2.f) / 2);
	
	// 输出圆尺寸
	cout << "vmax : "<< vmax << "  vmin : " << vmin << endl;
    const double hp2 = HALF_PATCH_SIZE*HALF_PATCH_SIZE;
    for (v = 0; v <= vmax; ++v) {
        umax[v] = cvRound(sqrt(hp2 - v * v));
		//cout << "umax[v] at " <<  v << " num is " <<  umax[v] << endl;
	}
	// ??
    // Make sure we are symmetric
	// umax = {15, 15, 15, 15, 14, 14, 14, 13, 13, 12, 11, 10, 9, 8, 6, 3};
	// v    = {0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12,13,14,15};
    for (v = HALF_PATCH_SIZE, v0 = 0; v >= vmin; --v)
    {
        while (umax[v0] == umax[v0 + 1]) ++v0;
        umax[v] = v0;
        ++v0;
    }	
	
	cout << "-------------- Initial Done ---------------" << endl;
}
*/

/**
 * 总体流程
 * 计算特征点
 * 计算描述向量
 * @param  _keypoints _descriptors 
 */

/*
void computeORB( InputArray _image, InputArray _mask, vector<KeyPoint>& _keypoints,
                      OutputArray _descriptors)
{ 
    if(_image.empty())
        return;

    Mat image = _image.getMat();
    assert(image.type() == CV_8UC1 );

    // Pre-compute the scale pyramid
    // 构建图像金字塔
    ComputePyramid(image);

    // 计算每层图像的兴趣点
    vector < vector<KeyPoint> > allKeypoints; // vector<vector<KeyPoint>>
    ComputeKeyPointsOld(allKeypoints);

    Mat descriptors;

    int nkeypoints = 0;
    for (int level = 0; level < nlevels; ++level)
        nkeypoints += (int)allKeypoints[level].size();
    if( nkeypoints == 0 )
        _descriptors.release();
    else
    {
        _descriptors.create(nkeypoints, 32, CV_8U);
        descriptors = _descriptors.getMat();
    }

    _keypoints.clear();
    _keypoints.reserve(nkeypoints);

    int offset = 0;
    for (int level = 0; level < nlevels; ++level)
    {
        vector<KeyPoint>& keypoints = allKeypoints[level];
        int nkeypointsLevel = (int)keypoints.size();

        if(nkeypointsLevel==0)
            continue;

        // preprocess the resized image 对图像进行高斯模糊
        Mat workingMat = mvImagePyramid[level].clone();
        GaussianBlur(workingMat, workingMat, Size(7, 7), 2, 2, BORDER_REFLECT_101);

        // Compute the descriptors 计算描述子
        Mat desc = descriptors.rowRange(offset, offset + nkeypointsLevel);
        computeDescriptors(workingMat, keypoints, desc, pattern);

        offset += nkeypointsLevel;

        // Scale keypoint coordinates
        if (level != 0)
        {
            float scale = mvScaleFactor[level]; //getScale(level, firstLevel, scaleFactor);
            for (vector<KeyPoint>::iterator keypoint = keypoints.begin(),
                 keypointEnd = keypoints.end(); keypoint != keypointEnd; ++keypoint)
                keypoint->pt *= scale;
        }
        // And add the keypoints to the output
        _keypoints.insert(_keypoints.end(), keypoints.begin(), keypoints.end());
    }
}
*/

void computeORB( InputArray _image, InputArray _mask, vector<KeyPoint>& _keypoints, OutputArray _descriptors) {
    
	if(_image.empty())
        return;

    Mat image = _image.getMat();
    assert(image.type() == CV_8UC1 );

    // Pre-compute the scale pyramid
    // 构建图像金字塔
	cout << "-------Compute Pyramid : " << nlevels << endl;
    ComputePyramid(image);
	
    // 计算每层图像的兴趣点及方向
	cout << "--- Compute Feature Points and Orientation ---" << endl;
    vector<vector<KeyPoint>> allKeypoints; // vector<vector<KeyPoint>>
    ComputeKeyPointsOld(allKeypoints);
	cout << "Image all Keypoints : " << allKeypoints.size() << endl;
	//cout << "all keypoint position: " << allKeypoint
	// 创建特征点数量大小的描述向量矩阵
    Mat descriptors;
    int nkeypoints = 0;
	
	// 求和
    for (int level = 0; level < nlevels; ++level)
        nkeypoints += (int)allKeypoints[level].size();
	// 创建描述子矩阵
    if( nkeypoints == 0 )
        _descriptors.release();
    else
    {
		// Note that _OutputArray::create() needs to be called before _OutputArray::getMat()
        _descriptors.create(nkeypoints, 32, CV_8U);
        descriptors = _descriptors.getMat();
    }
 
    // 更新特征点向量空间
    _keypoints.clear();
    _keypoints.reserve(nkeypoints);

    int offset = 0;
	
	cout << "----------- compute Descriptors ----------" << endl;
	// do what?
    for (int level = 0; level < nlevels; ++level)
    {		
		//cout << "level is : " << level << endl;
		
        vector<KeyPoint>& keypoints = allKeypoints[level];
        int nkeypointsLevel = (int)keypoints.size();

        if(nkeypointsLevel==0)
            continue;
	
		cout << "level_" << level << " nkeypointsLevel : "<< nkeypointsLevel << endl;
		// 赋值的矩阵和赋值矩阵之间空间独立
        Mat workingMat = mvImagePyramid[level].clone();
		Mat distMat;
        // preprocess the resized image 对图像进行高斯模糊
		// cnblog : tornadomeet
        // GaussianBlur(workingMat, distMat, Size(7, 7), 2, 2, BORDER_REFLECT_101);
		// 自定义7x7高斯模糊
		mGaussBlur(workingMat, distMat, 2);
		
		//imshow("GaussianBlur Image", distMat);
	
		// waitKey(0);
        // 计算每层特征点描述子
		// The method(rowRange) makes a new header for the specified row span of the matrix. 
		// Similarly to Mat::row() and Mat::col() , this is an O(1) operation.
        Mat desc = descriptors.rowRange(offset, offset + nkeypointsLevel);
        computeDescriptors(distMat, keypoints, desc, pattern);
		// 每一层特征点个数作为偏移位置
        offset += nkeypointsLevel;
		
		// cout << "将不同层的特征点坐标缩放到原图位置" << endl;
        // Scale level 1 and 2 keypoint coordinates
        if (level != 0)
        {
            float scale = mvScaleFactor[level]; //getScale(level, firstLevel, scaleFactor);
            for (vector<KeyPoint>::iterator keypoint = keypoints.begin(), keypointEnd = keypoints.end(); keypoint != keypointEnd; ++keypoint) {
                keypoint->pt *= scale;
			}
			//cout << "this level scale done " << endl;
        }
        
        // And add the keypoints to the output vector
        //  void insert (iterator position, InputIterator first, InputIterator last);
        // Copies of the elements in the range [first,last) are inserted at position (in the same order).
        _keypoints.insert(_keypoints.end(), keypoints.begin(), keypoints.end());
		

    }
    // 输出特征点向量
	//for (int i = 0; i < nkeypoints; i++)
	//	cout << i << "_keypoints : " << _keypoints[i].pt << " response :" << _keypoints[i].response << endl;
	
    cout << "------------End-------------" << endl;	
	
}

	
/**
 * 优化高斯模糊 :
 * @param  sigma
 */
void mGaussBlur(const Mat &src, Mat &dst, double sigma) {
	if(src.channels() != 1)
		return;
	
	dst.create(src.size(), src.type());
	
	uchar* srcData = src.data;
	uchar* dstData = dst.data;
	
	int center = (ksize-1) /2;
	//图像卷积运算,处理边缘
	for(int j = 0; j < src.cols; j++)
	{
		for(int i = 0; i < src.rows; i++)
		{
			double acc = 0;

			for(int m = -center, c = 0; m <= center; m++, c++)
			{
				for(int n = -center, r = 0; n <= center; n++, r++)
				{
					if((i+n) >=0 && (i+n) < src.rows && (j+m) >=0 && (j+m) < src.cols)
					{
						
						acc += *(srcData + src.step * (i+n) + src.channels() * (j+m)) * gaussianTemplate[c][r]; 
				
					}
				}
			}

			// 赋值给新图像
			*(dstData + dst.step * (i) + (j)) = (int)acc;
		}
	}
	
}



/**
 * 构建图像金字塔
 * @param image 输入图像
 */
void ComputePyramid(cv::Mat image)
{
	/*
	// 填不填边都一样,反正都是得到三张下采样图像
	for (int level = 0; level < nlevels; ++level)
    {
		float scale = mvInvScaleFactor[level];
		Size sz(cvRound((float)image.cols*scale), cvRound((float)image.rows*scale));
		
        if( level != 0 )
        {
			resize(mvImagePyramid[level-1], mvImagePyramid[level], sz, 0, 0, cv::INTER_LINEAR);
		}	
		else
		{
			mvImagePyramid[level] =  image;
		}
		// 
		
		cout << "mvImagePyramid-"<< level << " rows : "<< mvImagePyramid[level].rows << " cols : "<< mvImagePyramid[level].cols << endl;
		imshow("mvImagePyramid image ", mvImagePyramid[level]);
	
		waitKey(0);
		
		
	}
	*/
	
    for (int level = 0; level < nlevels; ++level)
    {
        float scale = mvInvScaleFactor[level];
		
		cout << "level " << level << " scale is " << scale << endl;
		// Size
        Size sz(cvRound((float)image.cols*scale), cvRound((float)image.rows*scale));
        Size wholeSize(sz.width + EDGE_THRESHOLD*2, sz.height + EDGE_THRESHOLD*2);
		
		cout << "sz is " << sz << endl;
		cout << "wholeSize is " << wholeSize << endl;
		// cv::Rect表示一个矩形区域
		// Rect(x,y,width,height)，x, y 为左上角坐标, width, height 则为长和宽
        Mat temp(wholeSize, image.type());
		
		// 定义一个Mat类型并给其设定ROI区域  
		// Mat 	operator() (const Rect &roi) const
		// 新的信息头，指向同一片数据空间
        mvImagePyramid[level] = temp(Rect(EDGE_THRESHOLD, EDGE_THRESHOLD, sz.width, sz.height));

		//imshow("temp image", temp);
		//waitKey(0);
		//imshow("mvImagePyramid first", mvImagePyramid[level]);
		//waitKey(0);
        // Compute the resized image
        if( level != 0 )
        {
			// dsize = Size(round(fx∗src.cols),round(fy∗src.rows))
			// 图像调整大小
			// void resize(InputArray src, OutputArray dst, 
			//				Size dsize, double fx=0, double fy=0, 
			//				int interpolation=INTER_LINEAR ); 
			// interpolation 这个是指定插值的方式
            resize(mvImagePyramid[level-1], mvImagePyramid[level], sz, 0, 0, cv::INTER_LINEAR);
			
			cout << "src image cols is " << mvImagePyramid[level-1].cols << "; rows is " << mvImagePyramid[level-1].rows << endl;
			// 扩充图像边界
			// Forms a border around an image.
			// void copyMakeBorder(InputArray src, OutputArray dst, int top, int bottom, int left, int right, int borderType, const Scalar& value=Scalar() )
			// The function copies the source image into the middle of the destination image
            copyMakeBorder(mvImagePyramid[level], temp, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD, BORDER_REFLECT_101+BORDER_ISOLATED);      
			
			// 金字塔图像
			//imshow("mvImagePyramid ", mvImagePyramid[level-1]);
			//waitKey(0);
			
			//imshow("temp", temp);
			//waitKey(0);
        }
        // 初始化
        else
        {
			////!< `gfedcb|abcdefgh|gfedcba` 
            copyMakeBorder(image, temp, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD, BORDER_REFLECT_101);   
			//imshow("temp level-0 ", temp);
			//waitKey(0);
			//cout << "temp level 0 row is " << temp.rows << "  col is : " << temp.cols << endl;			
        }
        
        cout << "mvImagePyramid-"<< level << " rows : "<< mvImagePyramid[level].rows << " cols : "<< mvImagePyramid[level].cols << endl;
		// final pyramid image
        //imshow("mvImagePyramid end", mvImagePyramid[level]);
		//waitKey(0);
    }
	
	
	
}	
	
	
/**
 * 计算特征点
 * @param allKeypoints 输出特征点
 */	
/*
void ComputeKeyPointsOld(std::vector<std::vector<KeyPoint> > &allKeypoints)
{
    allKeypoints.resize(nlevels);

    float imageRatio = (float)mvImagePyramid[0].cols/mvImagePyramid[0].rows;

    for (int level = 0; level < nlevels; ++level)
    {
        const int nDesiredFeatures = mnFeaturesPerLevel[level];

        const int levelCols = sqrt((float)nDesiredFeatures/(5*imageRatio));
        const int levelRows = imageRatio*levelCols;

        const int minBorderX = EDGE_THRESHOLD;
        const int minBorderY = minBorderX;
        const int maxBorderX = mvImagePyramid[level].cols-EDGE_THRESHOLD;
        const int maxBorderY = mvImagePyramid[level].rows-EDGE_THRESHOLD;

        const int W = maxBorderX - minBorderX;
        const int H = maxBorderY - minBorderY;
        const int cellW = ceil((float)W/levelCols);
        const int cellH = ceil((float)H/levelRows);

        const int nCells = levelRows*levelCols;
        const int nfeaturesCell = ceil((float)nDesiredFeatures/nCells);

        vector<vector<vector<KeyPoint> > > cellKeyPoints(levelRows, vector<vector<KeyPoint> >(levelCols));

        vector<vector<int> > nToRetain(levelRows,vector<int>(levelCols,0));
        vector<vector<int> > nTotal(levelRows,vector<int>(levelCols,0));
        vector<vector<bool> > bNoMore(levelRows,vector<bool>(levelCols,false));
        vector<int> iniXCol(levelCols);
        vector<int> iniYRow(levelRows);
        int nNoMore = 0;
        int nToDistribute = 0;


        float hY = cellH + 6;

        for(int i=0; i<levelRows; i++)
        {
            const float iniY = minBorderY + i*cellH - 3;
            iniYRow[i] = iniY;

            if(i == levelRows-1)
            {
                hY = maxBorderY+3-iniY;
                if(hY<=0)
                    continue;
            }

            float hX = cellW + 6;

            for(int j=0; j<levelCols; j++)
            {
                float iniX;

                if(i==0)
                {
                    iniX = minBorderX + j*cellW - 3;
                    iniXCol[j] = iniX;
                }
                else
                {
                    iniX = iniXCol[j];
                }


                if(j == levelCols-1)
                {
                    hX = maxBorderX+3-iniX;
                    if(hX<=0)
                        continue;
                }


                Mat cellImage = mvImagePyramid[level].rowRange(iniY,iniY+hY).colRange(iniX,iniX+hX);

                cellKeyPoints[i][j].reserve(nfeaturesCell*5);

                FAST(cellImage,cellKeyPoints[i][j],iniThFAST,true);

                if(cellKeyPoints[i][j].size()<=3)
                {
                    cellKeyPoints[i][j].clear();

                    FAST(cellImage,cellKeyPoints[i][j],minThFAST,true);
                }


                const int nKeys = cellKeyPoints[i][j].size();
                nTotal[i][j] = nKeys;

                if(nKeys>nfeaturesCell)
                {
                    nToRetain[i][j] = nfeaturesCell;
                    bNoMore[i][j] = false;
                }
                else
                {
                    nToRetain[i][j] = nKeys;
                    nToDistribute += nfeaturesCell-nKeys;
                    bNoMore[i][j] = true;
                    nNoMore++;
                }

            }
        }


        // Retain by score

        while(nToDistribute>0 && nNoMore<nCells)
        {
            int nNewFeaturesCell = nfeaturesCell + ceil((float)nToDistribute/(nCells-nNoMore));
            nToDistribute = 0;

            for(int i=0; i<levelRows; i++)
            {
                for(int j=0; j<levelCols; j++)
                {
                    if(!bNoMore[i][j])
                    {
                        if(nTotal[i][j]>nNewFeaturesCell)
                        {
                            nToRetain[i][j] = nNewFeaturesCell;
                            bNoMore[i][j] = false;
                        }
                        else
                        {
                            nToRetain[i][j] = nTotal[i][j];
                            nToDistribute += nNewFeaturesCell-nTotal[i][j];
                            bNoMore[i][j] = true;
                            nNoMore++;
                        }
                    }
                }
            }
        }

        vector<KeyPoint> & keypoints = allKeypoints[level];
        keypoints.reserve(nDesiredFeatures*2);

        const int scaledPatchSize = PATCH_SIZE*mvScaleFactor[level];

        // Retain by score and transform coordinates
        for(int i=0; i<levelRows; i++)
        {
            for(int j=0; j<levelCols; j++)
            {
                vector<KeyPoint> &keysCell = cellKeyPoints[i][j];
                KeyPointsFilter::retainBest(keysCell,nToRetain[i][j]);
                if((int)keysCell.size()>nToRetain[i][j])
                    keysCell.resize(nToRetain[i][j]);


                for(size_t k=0, kend=keysCell.size(); k<kend; k++)
                {
                    keysCell[k].pt.x+=iniXCol[j];
                    keysCell[k].pt.y+=iniYRow[i];
                    keysCell[k].octave=level;
                    keysCell[k].size = scaledPatchSize;
                    keypoints.push_back(keysCell[k]);
                }
            }
        }

        if((int)keypoints.size()>nDesiredFeatures)
        {
            KeyPointsFilter::retainBest(keypoints,nDesiredFeatures);
            keypoints.resize(nDesiredFeatures);
        }
    }

    // and compute orientations
    for (int level = 0; level < nlevels; ++level)
        computeOrientation(mvImagePyramid[level], allKeypoints[level], umax);
}

*/

void ComputeKeyPointsOld(vector<vector<KeyPoint> > &allKeypoints)
{
	// vector::resize 
	// Resizes the container so that it contains n elements.
    allKeypoints.resize(nlevels);

    float imageRatio = (float)mvImagePyramid[0].cols/mvImagePyramid[0].rows;
	cout << ">>>> imageRatio : " << imageRatio << endl;
	
    for (int level = 0; level < nlevels; ++level)
    {
		
		cout << "----- level_" << level << "--------" << endl;
        const int nDesiredFeatures = mnFeaturesPerLevel[level];
		// ?? 
        const int levelCols = sqrt((float)nDesiredFeatures/(5*imageRatio));
        const int levelRows = imageRatio*levelCols;

        const int minBorderX = EDGE_THRESHOLD;
        const int minBorderY = minBorderX;
        const int maxBorderX = mvImagePyramid[level].cols-EDGE_THRESHOLD;
        const int maxBorderY = mvImagePyramid[level].rows-EDGE_THRESHOLD;

        const int W = maxBorderX - minBorderX;
        const int H = maxBorderY - minBorderY;
        const int cellW = ceil((float)W/levelCols);
        const int cellH = ceil((float)H/levelRows);

        const int nCells = levelRows*levelCols;
        const int nfeaturesCell = ceil((float)nDesiredFeatures/nCells);

		cout << "nCells : "<< nCells << "   nfeaturesCell : " << nfeaturesCell << "  nDesiredFeatures : " << nDesiredFeatures << endl;
		cout << "levelCols : " << levelCols << "  levelRows : " << levelRows << endl;
		// 新增一维 levelRows
        vector<vector<vector<KeyPoint> > > cellKeyPoints(levelRows, vector<vector<KeyPoint>>(levelCols));
		// -- 按块分配参数 --
		// 每块要保留特征点个数
        vector<vector<int> >  nToRetain(levelRows,vector<int>(levelCols,0));
		// 记录每块要保留特征点个数
        vector<vector<int> >  nTotal(levelRows,vector<int>(levelCols,0));
        vector<vector<bool> > bNoMore(levelRows,vector<bool>(levelCols,false));
        vector<int> iniXCol(levelCols);
        vector<int> iniYRow(levelRows);
		// 层内有效
		// 没有达到分配特征点个数的Cell数
        int nNoMore = 0;
		// 要贡献出来的特征点个数
        int nToDistribute = 0;


        float hY = cellH + 6;
		
		// ?? 
        for(int i=0; i<levelRows; i++)
        {
            const float iniY = minBorderY + i*cellH - 3;
            iniYRow[i] = iniY;

            if(i == levelRows-1)
            {
                hY = maxBorderY+3-iniY;
                if(hY<=0)
                    continue;
            }
            
			float hX = cellW + 6;
			// 求每一层特征点
            for(int j=0; j<levelCols; j++)
            {
                float iniX;

                if(i==0)
                {
                    iniX = minBorderX + j*cellW - 3;
                    iniXCol[j] = iniX;
                }
                else
                {
                    iniX = iniXCol[j];
                }


                if(j == levelCols-1)
                {
                    hX = maxBorderX+3-iniX;
					// 跳过循环体剩余部分
                    if(hX<=0)
                        continue;
                }

				// rowRange(int startrow, int endrow) const
				// colRange(int startcol, int endcol) const
				// The method makes a new header for the specified row span of the matrix.
				// 分块图像
                Mat cellImage = mvImagePyramid[level].rowRange(iniY,iniY+hY).colRange(iniX,iniX+hX);
				cout << "iniY : "<< iniY << " hY : "<< hY << " iniX : "<< iniX << " hX : "<< hX << endl;
				// Request a change in capacity, but why 5?
                cellKeyPoints[i][j].reserve(nfeaturesCell*5);
				
				// 04-26-2018
				// imshow("cell Image 05-07", cellImage);
				// waitKey(0);
				
				// Detects corners using the FAST algorithm
				// void FAST(InputArray image, vector<KeyPoint>& keypoints, int threshold, bool nonmaxSuppression=true )
                FAST(cellImage,cellKeyPoints[i][j],iniThFAST,true);
				
				// 个数小于3
                if(cellKeyPoints[i][j].size()<=3)
                {
                    cellKeyPoints[i][j].clear();

                    FAST(cellImage,cellKeyPoints[i][j],minThFAST,true);
                }

				// 得到每个cell的特征点个数
                const int nKeys = cellKeyPoints[i][j].size();
                nTotal[i][j] = nKeys;
				// cout << "nTotal[i][j] : "<< nTotal[i][j] << " nKeys "<< nKeys << endl;
				// 比理论值大
                if(nKeys>nfeaturesCell)
                {
                    nToRetain[i][j] = nfeaturesCell;
                    bNoMore[i][j] = false;
                }
                // 比理论值少了多少
                else
                {
                    nToRetain[i][j] = nKeys;
                    nToDistribute += nfeaturesCell-nKeys;
                    bNoMore[i][j] = true;
                    nNoMore++;
                }

            } // end for j
            
        } // end for i
        
		// 完成分层特征点提取
		
		// test : {false = 0, true = 1}
		cout << "True is " << true << "  false is " << false << endl;
		
		cout << "nToDistribute is " << nToDistribute << "  nNoMore is " << nNoMore << endl;
        // Retain by score
		// 筛选特征点
		// 直到没有多出来的特征点盒子 没有多出来的特征点
		// 这里有Bug
        while(nToDistribute>0 && nNoMore<nCells)
        {
			// 调整Cell特征点个数
            int nNewFeaturesCell = nfeaturesCell + ceil((float)nToDistribute/(nCells-nNoMore));
			cout << "nNewFeaturesCell num is " << nNewFeaturesCell << endl;
			// 没达到指定个数能贡献出来的特征点个数
            nToDistribute = 0;
			// ?? 
            for(int i=0; i<levelRows; i++)
            {
                for(int j=0; j<levelCols; j++)
                {
					// bNoMore == false 
					// 直到当前cell里的特征点个数小于新设定的值
                    if(!bNoMore[i][j])
                    {
						// 这不是废话
						// 调整特征点分布
						// 已经贡献完了，刚好设定的点数
                        if(nTotal[i][j]>nNewFeaturesCell)
                        {
                            nToRetain[i][j] = nNewFeaturesCell;
                            bNoMore[i][j] = false;
                        }
                        else
                        {
							// 这还不是nkeys吗
                            nToRetain[i][j] = nTotal[i][j];
                            nToDistribute += nNewFeaturesCell-nTotal[i][j];
                            bNoMore[i][j] = true;
                            nNoMore++;
                        }
                    } // end if
                    // 预期值和得到值
                    cout << "i:" << i << "  j:" << j << endl;
					cout << "bNoMore[i][j] : " << bNoMore[i][j] << endl;
                    cout << "nNewFeaturesCell : " << nNewFeaturesCell << " nToRetain[i][j] : " << nToRetain[i][j] << "  nTotal[i][j] : "<< nTotal[i][j] << endl;
                    
                } // end for
            } // end for
            
            cout << "nToDistribute is " << nToDistribute << "  nNoMore is " << nNoMore << endl;
			
        }
        
		// 这一层分配完成
		
        vector<KeyPoint> & keypoints = allKeypoints[level];
		// vector::reserve
		// Requests that the vector capacity be at least enough to contain n elements.
		// *2 ??
        keypoints.reserve(nDesiredFeatures*2);

        const int scaledPatchSize = PATCH_SIZE*mvScaleFactor[level];

        // Retain by score and transform coordinates
		// nToRetain and keysCell -> keypoints
        for(int i=0; i<levelRows; i++)
        {
            for(int j=0; j<levelCols; j++)
            {
                vector<KeyPoint> &keysCell = cellKeyPoints[i][j];
				
				cout << "-- keysCell size : " << keysCell.size() << " nToRetain[i][j] : " << nToRetain[i][j] << endl;
				
				// KeyPointsFilter ()
				// A class filters a vector of keypoints.
				// void retainBest (std::vector< KeyPoint > &keypoints, int npoints)
				// takes keypoints and culls them by the response
				// 将特征点消减到新的特征点数量
				// cull to the final desired level, using the new Harris scores or the original FAST scores.  
				// 按Harris强度排序，保留前featuresNum个 
                //KeyPointsFilter::retainBest(keysCell, nToRetain[i][j]);
				// Directed by Belong 05-10-2018
				mRetainBest(keysCell, nToRetain[i][j]);
				
				cout << "** keysCell size : " << keysCell.size() << " nToRetain[i][j] : " << nToRetain[i][j] << endl;
				// 这有用吗
                if((int)keysCell.size()>nToRetain[i][j])
                    keysCell.resize(nToRetain[i][j]);

				// 保存特征点信息
                for(size_t k=0, kend=keysCell.size(); k<kend; k++)
                {
					// 根据cell计算特征点坐标
                    keysCell[k].pt.x+=iniXCol[j];
                    keysCell[k].pt.y+=iniYRow[i];
                    keysCell[k].octave=level;
                    keysCell[k].size = scaledPatchSize;
                    keypoints.push_back(keysCell[k]);
                }
            }
        }
		
		// 剔除超过设定个数
        if((int)keypoints.size()>nDesiredFeatures)
        {
			// keep best
            // KeyPointsFilter::retainBest(keypoints,nDesiredFeatures);
			// Directed by Belong 05-10-2018
			mRetainBest(keypoints, nDesiredFeatures);
            keypoints.resize(nDesiredFeatures);
        }
    } // end level

    // compute every level keypoint orientations
    for (int level = 0; level < nlevels; ++level)
        computeOrientation(mvImagePyramid[level], allKeypoints[level], umax);
}



/*
 * 保留指定个数特征点数
 * @param npoints
 */
void mRetainBest( std::vector<KeyPoint>& keypoints, int mNpoints ) {
	if((int)keypoints.size() > mNpoints)
	keypoints.resize(mNpoints);
}

/**
 * 计算特征点主方向
 * @param desc 输出特征点主方向
 */	

void computeOrientation(const Mat& image, vector<KeyPoint>& keypoints, const vector<int>& umax)
{
	// cout << "-------- 计算特征点主方向 --------"<< endl;
	// 通过迭代器遍历元素
	// iterator is a pointer: A pointer can point to elements in an array
    for (vector<KeyPoint>::iterator keypoint = keypoints.begin(), keypointEnd = keypoints.end(); keypoint != keypointEnd; ++keypoint)
    {
		// 角度
		keypoint->angle = IC_Angle(image, keypoint->pt, umax);
    }
}


/**
 * 计算特征点主方向
 * @param pt 输出特征点方向
 */	
float IC_Angle(const Mat& image, Point2f pt,  const vector<int> & u_max)
{
	// 
    int m_01 = 0, m_10 = 0;
	// 特征点位置
    const uchar* center = &image.at<uchar> (cvRound(pt.y), cvRound(pt.x));

    // Treat the center line differently, v=0
    for (int u = -HALF_PATCH_SIZE; u <= HALF_PATCH_SIZE; ++u)
        m_10 += u * center[u];

    // Go line by line in the circuI853lar patch
    int step = (int)image.step1();
    for (int v = 1; v <= HALF_PATCH_SIZE; ++v)
    {
        // Proceed over the two lines
        int v_sum = 0;
        int d = u_max[v];
        for (int u = -d; u <= d; ++u)
        {
            int val_plus = center[u + v*step];
			int val_minus = center[u - v*step];
            v_sum += (val_plus - val_minus);
            m_10 += u * (val_plus + val_minus);
        }
        m_01 += v * v_sum;
    }
    
	// 输入一个2维向量，计算这个向量的方向，以度为单位（范围是0度---360度），精度是0.3度
    return fastAtan2((float)m_01, (float)m_10);
}



/**
 * 计算特征点描述子
 * @param desc 输出特征点描述子
 */	

static void computeOrbDescriptor(const KeyPoint& kpt,
                                 const Mat& img, const Point* pattern,
                                 uchar* desc)
{
	// 弧度转角度
    float angle = (float)kpt.angle*factorPI;
	// x-y 分量
    float a = (float)cos(angle), b = (float)sin(angle);
	// 指针指向一个常量对象
	// 特征点位置作为中心点
    const uchar* center = &img.at<uchar>(cvRound(kpt.pt.y), cvRound(kpt.pt.x));
	// Returns a normalized step same to stride
    const int step = (int)img.step;	// 640 533 444 

	// cout << "center" << *center << "  step" << step << endl;
	// 获取特征点为中心主方向PITCH内的像素值
    // #define GET_VALUE(idx) center[cvRound(pattern[idx].x*b + pattern[idx].y*a)*step + cvRound(pattern[idx].x*a - pattern[idx].y*b)]
	//float xb = pattern[idx].x*b + pattern[idx].y*a;
	//float yb = pattern[idx].x*a - pattern[idx].y*b;
	#define GET_VALUE(idx) img.at<uchar>(cvRound(kpt.pt.y+pattern[idx].x*a - pattern[idx].y*b), cvRound(kpt.pt.x+pattern[idx].x*b + pattern[idx].y*a))
	
	// center[<0] ?? 
	// cout << "111111: " << cvRound(pattern[7].x*b + pattern[7].y*a) << "   222222 : " << cvRound(pattern[7].x*a - pattern[7].y*b) << endl;
	// 得到该点的512位宽的描述向量，并保存
    for (int i = 0; i < 32; ++i, pattern += 16)
    {
        uchar t0, t1, val = 0;
        t0 = GET_VALUE(0); t1 = GET_VALUE(1);
        val = t0 < t1;
        t0 = GET_VALUE(2); t1 = GET_VALUE(3);
        val |= (t0 < t1) << 1;
        t0 = GET_VALUE(4); t1 = GET_VALUE(5);
        val |= (t0 < t1) << 2;
        t0 = GET_VALUE(6); t1 = GET_VALUE(7);
        val |= (t0 < t1) << 3;
        t0 = GET_VALUE(8); t1 = GET_VALUE(9);
        val |= (t0 < t1) << 4;
        t0 = GET_VALUE(10); t1 = GET_VALUE(11);
        val |= (t0 < t1) << 5;
        t0 = GET_VALUE(12); t1 = GET_VALUE(13);
        val |= (t0 < t1) << 6;
        t0 = GET_VALUE(14); t1 = GET_VALUE(15);
        val |= (t0 < t1) << 7;

        desc[i] = (uchar)val;
    }

    #undef GET_VALUE
}


/**
 * 计算特征点描述子
 * @param descriptors 输出特征点描述子
 */	

void computeDescriptors(const Mat& image, vector<KeyPoint>& keypoints, Mat& descriptors, const vector<Point>& pattern)
{
	// cout << "-------- 特征点描述子 -------" << endl;
	// 逐层计算
	// 初始化矩阵 row = (int)keypoints.size();  col = 32*uchar;
    descriptors = Mat::zeros((int)keypoints.size(), 32, CV_8UC1);
	// 逐点计算
    for (size_t i = 0; i < keypoints.size(); i++)
		// uchar* cv::Mat::ptr : Returns a pointer to the specified matrix row.
        computeOrbDescriptor(keypoints[i], image, &pattern[0], descriptors.ptr((int)i) );
}
