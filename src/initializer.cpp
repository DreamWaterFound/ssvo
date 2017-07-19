#include <opencv2/core.hpp>
#include <opencv2/video.hpp>
#include <opencv2/opencv.hpp>
#include "global.hpp"

#include "initializer.hpp"
#include <opencv/cxeigen.hpp>

namespace ssvo{

InitResult Initializer::addFirstFrame(FramePtr frame_ref)
{
    //! reset
    pts_ref_.clear();
    pts_cur_.clear();
    p3ds_.clear();
    disparities_.clear();
    inliers_.release();

    //! check corner number of first image
    if(frame_ref->kps_.size() < Config::initMinCorners())
    {
        SSVO_WARN_STREAM("[INIT] First image has too less corners !!!");
        return RESET;
    }

    frame_ref_ = frame_ref;
    cv::KeyPoint::convert(frame_ref_->kps_, pts_ref_);

    //! set initial flow
    pts_cur_.insert(pts_cur_.begin(), pts_ref_.begin(), pts_ref_.end());

    return SUCCESS;
}

InitResult Initializer::addSecondFrame(FramePtr frame_cur)
{
    double t1 = (double)cv::getTickCount();
    frame_cur_ = frame_cur;
    kltTrack(frame_ref_->img_pyr_[0], frame_cur_->img_pyr_[0], pts_ref_, pts_cur_, disparities_);
    inliers_ = cv::Mat::ones(pts_ref_.size(), 1, CV_8UC1);

    SSVO_INFO_STREAM("[INIT] KLT tracking points: " << disparities_.size());
    if(disparities_.size() < Config::initMinTracked())
        return RESET;

    double t2 = (double)cv::getTickCount();
    double disparity = 0.0;
    for_each(disparities_.begin(), disparities_.end(), [&](double &d){disparity+=d;});
    disparity /= disparities_.size();

    SSVO_INFO_STREAM("[INIT] Avage disparity: " << disparity);
    if(disparity < Config::initMinDisparity())
        return FAILURE;

    double t3 = (double)cv::getTickCount();
    //! geometry check
    cv::Mat F;
    int inliers_count = Fundamental::findFundamentalMat(pts_ref_, pts_cur_, F, inliers_, Config::initSigma(), Config::initMaxRansacIters());
    //F  = cv::findFundamentalMat(pts_ref_, pts_cur_, inliers_, cv::FM_RANSAC, Config::initSigma()*3);//, Config::initMaxRansacIters());
    //F.convertTo(F, CV_32FC1);
    //int inliers_count = cv::countNonZero(inliers_);
    SSVO_INFO_STREAM("[INIT] Inliers after Fundamental Maxtrix RANSCA check: " << inliers_count);
    if(inliers_count < Config::initMinInliers())
        return FAILURE;

    double t4 = (double)cv::getTickCount();
    cv::Mat R1, R2, t;
    cv::Mat K1= frame_ref_->cam_->cvK();
    cv::Mat K2= frame_cur_->cam_->cvK();
    cv::Mat E = K1.t() * F * K2;
    Fundamental::decomposeEssentialMat(E, R1, R2, t);

    double t5 = (double)cv::getTickCount();
    cv::Mat T;
    cv::Mat P3Ds;
    bool succeed = findBestRT(R1, R2, t, K1, K2, pts_ref_, pts_cur_, inliers_, P3Ds, T);
    if(!succeed)
        return FAILURE;

    const int n = pts_ref_.size();
    std::vector<cv::Point2f> pts_ref;
    std::vector<cv::Point2f> pts_cur;
    pts_ref.reserve(n);
    pts_cur.reserve(n);
    p3ds_.reserve(n);

    const uchar* inliers_ptr = inliers_.ptr<uchar>(0);
    float* P3Ds_ptr = P3Ds.ptr<float>(0);
    int strick = P3Ds.cols;
    for(int i = 0; i < n; ++i, ++P3Ds_ptr)
    {
        if(!inliers_ptr[i])
            continue;

        pts_ref.push_back(pts_ref_[i]);
        pts_cur.push_back(pts_cur_[i]);
        p3ds_.push_back(Vector3f(*P3Ds_ptr, P3Ds_ptr[strick], P3Ds_ptr[strick*2]));
    }
    pts_ref_ = pts_ref;
    pts_cur_ = pts_cur;

    SSVO_INFO_STREAM("[INIT] Inliers after cheirality check: " << cv::countNonZero(inliers_));

    double t6 = (double)cv::getTickCount();
    std::cout << "Time: " << (t2-t1)/cv::getTickFrequency() << " "
                          << (t3-t2)/cv::getTickFrequency() << " "
                          << (t4-t3)/cv::getTickFrequency() << " "
                          << (t5-t4)/cv::getTickFrequency() << " "
                          << (t6-t5)/cv::getTickFrequency() << std::endl;

    SSVO_INFO_STREAM("[INIT] Initialization succeed!");

    return SUCCESS;
}

void Initializer::getTrackedPoints(std::vector<cv::Point2f>& pts_ref, std::vector<cv::Point2f>& pts_cur)
{
    const int n = pts_ref_.size();
    pts_ref.reserve(n);
    pts_cur.reserve(n);

    const uchar* inliers_ptr = inliers_.ptr<uchar>(0);
    for(int i = 0; i < n; ++i)
    {
        if(!inliers_ptr[i])
            continue;

        pts_ref.push_back(pts_ref_[i]);
        pts_cur.push_back(pts_cur_[i]);
    }
}

void Initializer::kltTrack(const cv::Mat& img_ref, const cv::Mat& img_cur, std::vector<cv::Point2f>& pts_ref, std::vector<cv::Point2f>& pts_cur, std::vector<double>& disparities)
{
    const int klt_win_size = 21;
    const int klt_max_iter = 30;
    const double klt_eps = 0.001;

    std::vector<float> error;
    std::vector<uchar> status;

    cv::TermCriteria termcrit(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, klt_max_iter, klt_eps);

    cv::calcOpticalFlowPyrLK(
        img_ref, img_cur,
        pts_ref, pts_cur,
        status, error,
        cv::Size(klt_win_size, klt_win_size), 3,
        termcrit, cv::OPTFLOW_USE_INITIAL_FLOW
    );

    std::vector<cv::Point2f>::iterator pt_ref_it = pts_ref.begin();
    std::vector<cv::Point2f>::iterator pt_cur_it = pts_cur.begin();

    disparities.clear();
    disparities.reserve(pts_ref.size());
    int size = pts_ref.size();
    for(int i = 0; pt_ref_it!= pts_ref.end(); ++i)
    {
        if(!status[i])
        {
            *pt_ref_it = pts_ref.back();//pts_ref.erase(pt_ref_it);
            *pt_cur_it = pts_cur.back();//erase(pt_cur_it);
            pts_ref.pop_back();
            pts_cur.pop_back();
            status[i] = status[--size];
            continue;
        }

        double dx = pt_ref_it->x - pt_cur_it->x;
        double dy = pt_ref_it->y - pt_cur_it->y;
        disparities.push_back(sqrt(dx*dx + dy*dy));
        ++pt_ref_it, ++pt_cur_it;
    }
}

bool Initializer::findBestRT(const cv::Mat& R1, const cv::Mat& R2, const cv::Mat& t, const cv::Mat& K1, const cv::Mat& K2,
                             const std::vector<cv::Point2f>& pts1, const std::vector<cv::Point2f>& pts2, cv::Mat& mask, cv::Mat& P3Ds, cv::Mat& T)
{
    assert(pts1.size() == pts2.size());
    if(mask.empty())
        mask = cv::Mat(pts1.size(), 1, CV_8UC1, cv::Scalar(255));

    //! P = K[R|t]
    cv::Mat P0 = cv::Mat::eye(3, 4, R1.type());
    cv::Mat P1(3, 4, R1.type(), cv::Scalar(0)), P2(3, 4, R1.type(), cv::Scalar(0)), P3(3, 4, R1.type(), cv::Scalar(0)), P4(3, 4, R1.type(), cv::Scalar(0));

    //! P0 = K[I|0]
    K1.copyTo(P0.rowRange(0,3).colRange(0,3));

    //! P1 = K[R1|t]
    P1.rowRange(0,3).colRange(0,3) = R1 * 1.0;
    P1.rowRange(0,3).col(3) = t * 1.0;
    P1 = K2*P1;

    //! P2 = K[R2|t]
    P2.rowRange(0,3).colRange(0,3) = R2 * 1.0;
    P2.rowRange(0,3).col(3) = t * 1.0;
    P2 = K2*P2;

    //! P3 = K[R1|-t]
    P3.rowRange(0,3).colRange(0,3) = R1 * 1.0;
    P3.rowRange(0,3).col(3) = t * -1.0;
    P3 = K2*P3;

    //! P4 = K[R2|-t]
    P4.rowRange(0,3).colRange(0,3) = R2 * 1.0;
    P4.rowRange(0,3).col(3) = t * -1.0;
    P4 = K2*P4;

    //! Do the cheirality check, and remove points too far away
    const float max_dist = 50.0;
    cv::Mat P3Ds1;
    triangulate(P0, P1, pts1, pts2, mask, P3Ds1);
    cv::Mat mask1 = mask.t();
    mask1 &= (P3Ds1.row(2) > 0) & (P3Ds1.row(2) < max_dist);
    P3Ds1 = P1*P3Ds1;
    mask1 &= (P3Ds1.row(2) > 0) & (P3Ds1.row(2) < max_dist);

    cv::Mat P3Ds2;
    triangulate(P0, P2, pts1, pts2, mask, P3Ds2);
    cv::Mat mask2 = mask.t();
    mask2 &= (P3Ds2.row(2) > 0) & (P3Ds2.row(2) < max_dist);
    P3Ds2 = P2*P3Ds2;
    mask2 &= (P3Ds2.row(2) > 0) & (P3Ds2.row(2) < max_dist);

    cv::Mat P3Ds3;
    triangulate(P0, P3, pts1, pts2, mask, P3Ds3);
    cv::Mat mask3 = mask.t();
    mask3 &= (P3Ds3.row(2) > 0) & (P3Ds3.row(2) < max_dist);
    P3Ds3 = P3*P3Ds3;
    mask3 &= (P3Ds3.row(2) > 0) & (P3Ds3.row(2) < max_dist);

    cv::Mat P3Ds4;
    triangulate(P0, P4, pts1, pts2, mask, P3Ds4);
    cv::Mat mask4 = mask.t();
    mask4 &= (P3Ds4.row(2) > 0) & (P3Ds4.row(2) < max_dist);
    P3Ds4 = P4*P3Ds4;
    mask4 &= (P3Ds4.row(2) > 0) & (P3Ds4.row(2) < max_dist);

    int nGood0 = cv::countNonZero(mask);
    int nGood1 = cv::countNonZero(mask1);
    int nGood2 = cv::countNonZero(mask2);
    int nGood3 = cv::countNonZero(mask3);
    int nGood4 = cv::countNonZero(mask4);

    int maxGood = MAX(MAX(nGood1, nGood2), MAX(nGood3, nGood4));
    //! for normal, other count is 0
    if(maxGood < 0.9*nGood0)
    {
        SSVO_WARN_STREAM("[INIT] Less than 90% inliers after cheirality check!!!");
        return false;
    }

    T = cv::Mat(3, 4, R1.type());
    if(maxGood == nGood1)
    {
        mask = mask1.t();
        P3Ds = P3Ds1.clone();
        T.rowRange(0,3).colRange(0,3) = R1.clone();
        T.rowRange(0,3).col(3) = t;
    }
    else if(maxGood == nGood2)
    {
        mask = mask2.t();
        P3Ds = P3Ds2.clone();
        T.rowRange(0,3).colRange(0,3) = R2.clone();
        T.rowRange(0,3).col(3) = t;
    }
    else if(maxGood == nGood3)
    {
        mask = mask3.t();
        P3Ds = P3Ds3.clone();
        T.rowRange(0,3).colRange(0,3) = R1.clone();
        T.rowRange(0,3).col(3) = t * -1.0;
    }
    else if(maxGood == nGood4)
    {
        mask = mask4.t();
        P3Ds = P3Ds4.clone();
        T.rowRange(0,3).colRange(0,3) = R2.clone();
        T.rowRange(0,3).col(3) = t * -1.0;
    }

    return true;
}

#if 0
void Initializer::triangulate(const cv::Mat& P1, const cv::Mat& P2, const std::vector<cv::Point2f>& pts1, const std::vector<cv::Point2f>& pts2, cv::Mat& mask, cv::Mat& P3D)
{
    const int n = pts1.size();
    assert(n == pts2.size());
    assert(mask.cols == 1 && mask.rows == n);
    uchar *mask_ptr = mask.ptr<uchar>(0);
    P3D = cv::Mat::zeros(4, n, CV_32FC1);
    for(int i = 0; i < n; ++i)
    {
        cv::Mat wP;

        if(mask_ptr[i])
            triangulate(P1, P2, pts1[i], pts2[i], wP);
        else
            wP = cv::Mat::zeros(4,1,CV_32FC1);
        wP.copyTo(P3D.col(i));
    }
}
#else
void Initializer::triangulate(const cv::Mat& P1, const cv::Mat& P2, const std::vector<cv::Point2f>& pts1, const std::vector<cv::Point2f>& pts2, cv::Mat& mask, cv::Mat& P3D)
{
    const int n = pts1.size();
    assert(n == pts2.size());
    assert(mask.cols == 1 && mask.rows == n);
    uchar *mask_ptr = mask.ptr<uchar>(0);
    MatrixXf eP1, eP2;
    cv::cv2eigen(P1, eP1);
    cv::cv2eigen(P2, eP2);
    MatrixXf eP3D = MatrixXf::Zero(4, n);
    P3D = cv::Mat::zeros(4, n, CV_32FC1);
    for(int i = 0; i < n; ++i)
    {
        Vector4f wP = Vector4f::Zero();

        if(mask_ptr[i]) {
            triangulate(eP1, eP2, pts1[i], pts2[i], wP);
        }

        eP3D.col(i) = wP;

    }
    cv::eigen2cv(eP3D, P3D);
}
#endif

void Initializer::triangulate(const cv::Mat& P1, const cv::Mat& P2, const cv::Point2f& pt1, const cv::Point2f& pt2, cv::Mat& P3D)
{
    cv::Mat A(4,4,CV_32F);

    A.row(0) = pt1.x*P1.row(2)-P1.row(0);
    A.row(1) = pt1.y*P1.row(2)-P1.row(1);
    A.row(2) = pt2.x*P2.row(2)-P2.row(0);
    A.row(3) = pt2.y*P2.row(2)-P2.row(1);

    cv::Mat U, W, Vt;
    cv::SVD::compute(A, W, U, Vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
    P3D = Vt.row(3).t();
    P3D = P3D/P3D.at<float>(3);
}

void Initializer::triangulate(const MatrixXf& P1, const MatrixXf& P2, const cv::Point2f& pt1, const cv::Point2f& pt2, Vector4f& P3D)
{
    MatrixXf A(4,4);
    A.row(0) = pt1.x*P1.row(2)-P1.row(0);
    A.row(1) = pt1.y*P1.row(2)-P1.row(1);
    A.row(2) = pt2.x*P2.row(2)-P2.row(0);
    A.row(3) = pt2.y*P2.row(2)-P2.row(1);

    JacobiSVD<MatrixXf> svd(A, ComputeThinV);
    MatrixXf V = svd.matrixV();

    P3D = V.col(3);
    P3D = P3D/P3D(3);
}

int Fundamental::findFundamentalMat(const std::vector<cv::Point2f>& pts_prev, const std::vector<cv::Point2f>& pts_next, cv::Mat &F,
                                    cv::Mat& inliers, float sigma, int max_iterations)
{
    assert(pts_prev.size() == pts_next.size());

    return runRANSAC(pts_prev, pts_next, F, inliers, sigma, max_iterations);
}

#if 1
void Fundamental::run8point(const std::vector<cv::Point2f>& pts_prev, const std::vector<cv::Point2f>& pts_next, cv::Mat& F)
{
    const int N = pts_prev.size();
    assert(N >= 8);

    std::vector<cv::Point2f> pts_prev_norm;
    std::vector<cv::Point2f> pts_next_norm;
    cv::Mat T1, T2;
    Normalize(pts_prev, pts_prev_norm, T1);
    Normalize(pts_next, pts_next_norm, T2);

    cv::Mat A(N, 9, CV_32F);
    for(int i = 0; i < N; ++i)
    {
        const float u1 = pts_prev_norm[i].x;
        const float v1 = pts_prev_norm[i].y;
        const float u2 = pts_next_norm[i].x;
        const float v2 = pts_next_norm[i].y;
        float* a = A.ptr<float>(i);

        a[0] = u2*u1;
        a[1] = u2*v1;
        a[2] = u2;
        a[3] = v2*u1;
        a[4] = v2*v1;
        a[5] = v2;
        a[6] = u1;
        a[7] = v1;
        a[8] = 1;
    }

    cv::Mat U, W, vt;

    cv::eigen(A.t()*A, W, vt);

    cv::Mat Fpre = vt.row(8).reshape(0, 3);

    cv::SVDecomp(Fpre, W, U, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    W.at<float>(2) = 0;

    cv::Mat F_norm = U*cv::Mat::diag(W)*vt;

    F = T2.t()*F_norm*T1;
    float F22 = F.at<float>(2, 2);
    if(fabs(F22) > FLT_EPSILON)
        F /= F22;
}
#else
void Fundamental::run8point(const std::vector<cv::Point2f>& pts_prev, const std::vector<cv::Point2f>& pts_next, cv::Mat& F)
{
    const int N = pts_prev.size();
    assert(N >= 8);

    std::vector<cv::Point2f> pts_prev_norm;
    std::vector<cv::Point2f> pts_next_norm;
    Matrix3f T1, T2;
    Normalize(pts_prev, pts_prev_norm, T1);
    Normalize(pts_next, pts_next_norm, T2);

    MatrixXf A(N,9);
    for(int i = 0; i < N; ++i)
    {
        const float u1 = pts_prev_norm[i].x;
        const float v1 = pts_prev_norm[i].y;
        const float u2 = pts_next_norm[i].x;
        const float v2 = pts_next_norm[i].y;

        A.row(i) <<  u2*u1, u2*v1, u2, v2*u1, v2*v1, v2, u1, v1, 1;
    }

    JacobiSVD<MatrixXf> svd(A, ComputeFullV);
    MatrixXf Va = svd.matrixV();

    VectorXf Fv = Va.col(8);
    Matrix3f Ft(Fv.data());

    JacobiSVD<MatrixXf> svd1(Ft.transpose(), ComputeFullV|ComputeFullU);
    MatrixXf V = svd1.matrixV();
    MatrixXf U = svd1.matrixU();
    Vector3f S = svd1.singularValues();
    S(2) = 0;
    DiagonalMatrix<float, Dynamic> W(S);

    Matrix3f Fn = U * W * V.transpose();

    MatrixXf FF = T2.transpose()*Fn*T1;
    float F22 = FF(2, 2);
    if(fabs(F22) > FLT_EPSILON)
        FF /= F22;

    cv::eigen2cv(FF, F);

}
#endif

int Fundamental::runRANSAC(const std::vector<cv::Point2f>& pts_prev, const std::vector<cv::Point2f>& pts_next, cv::Mat& F, cv::Mat& inliers, float sigma, int max_iterations)
{
    const int N = pts_prev.size();
    const int modelPoints = 8;
    assert(N >= modelPoints);

    const double threshold = 3.841*sigma*sigma;
    const int max_iters = MIN(MAX(max_iterations, 1), 1000);

    std::vector<int> total_points;
    for(int i = 0; i < N; ++i)
    {
        total_points.push_back(i);
    }

    std::vector<cv::Point2f> pts1(modelPoints);
    std::vector<cv::Point2f> pts2(modelPoints);
    std::vector<cv::Point2f> pts1_norm;
    std::vector<cv::Point2f> pts2_norm;
    cv::Mat F_temp;
    inliers = cv::Mat::zeros(N, 1, CV_8UC1);
    int max_inliers = 0;
    int niters = max_iters;
    for(int iter = 0; iter < niters; iter++)
    {
        std::vector<int> points = total_points;
        for(int i = 0; i < modelPoints; ++i)
        {
            int randi = Rand(0, points.size()-1);
            pts1[i] = pts_prev[points[randi]];
            pts2[i] = pts_next[points[randi]];

            points[randi] = points.back();
            points.pop_back();
        }

        run8point(pts1, pts2, F_temp);

        int inliers_count = 0;
        cv::Mat inliers_temp = cv::Mat::zeros(N, 1, CV_8UC1);
        uchar* inliers_ptr = inliers_temp.ptr<uchar>(0);
        for(int n = 0; n < N; ++n)
        {
            float error1, error2;
            computeErrors(pts_prev[n], pts_next[n], F_temp.ptr<float>(0), error1, error2);

            const float error = MAX(error1, error2);

            if(error < threshold)
            {
                inliers_ptr[n] = 0xff;
                //inliers_temp.at<uchar>(n, 0) = 255;
                inliers_count++;
            }
        }

        if(inliers_count > max_inliers)
        {
            max_inliers = inliers_count;
            inliers = inliers_temp.clone();

            if(inliers_count < N)
            {
                //! N = log(1-p)/log(1-omega^s)
                //! p = 99%
                //! number of set: s = 8
                //! omega = inlier points / total points
                const static double num = log(1 - 0.99);
                const double omega = inliers_count*1.0 / N;
                const double denom = log(1 - pow(omega, modelPoints));

                niters = (denom >= 0 || -num >= max_iters*(-denom)) ? max_iters : round(num / denom);
            }
            else
                break;
        }

    }//! iterations

    pts1.clear();
    pts2.clear();
    for(int n = 0; n < N; ++n)
    {
        if(0 == inliers.at<uchar>(n, 0))
        {
            continue;
        }

        pts1.push_back(pts_prev[n]);
        pts2.push_back(pts_next[n]);
    }

    run8point(pts1, pts2, F);

    return max_inliers;
}

void Fundamental::Normalize(const std::vector<cv::Point2f>& pts, std::vector<cv::Point2f>& pts_norm, cv::Mat& T)
{
    const int N = pts.size();
    if(N == 0)
        return;

    pts_norm.resize(N);

    cv::Point2f mean(0,0);
    for(int i = 0; i < N; ++i)
    {
        mean += pts[i];
    }
    mean = mean/N;

    cv::Point2f mean_dev(0,0);

    for(int i = 0; i < N; ++i)
    {
        pts_norm[i] = pts[i] - mean;

        mean_dev.x += fabs(pts_norm[i].x);
        mean_dev.y += fabs(pts_norm[i].y);
    }
    mean_dev /= N;

    const float scale_x = 1.0/mean_dev.x;
    const float scale_y = 1.0/mean_dev.y;

    for(int i=0; i<N; i++)
    {
        pts_norm[i].x *= scale_x;
        pts_norm[i].y *= scale_y;
    }

    T = cv::Mat::eye(3,3,CV_32F);
    T.at<float>(0,0) = scale_x;
    T.at<float>(1,1) = scale_y;
    T.at<float>(0,2) = -mean.x*scale_x;
    T.at<float>(1,2) = -mean.y*scale_y;
}

void Fundamental::Normalize(const std::vector<cv::Point2f>& pts, std::vector<cv::Point2f>& pts_norm, Matrix3f& T)
{
    const int N = pts.size();
    if(N == 0)
        return;

    pts_norm.resize(N);

    cv::Point2f mean(0,0);
    for(int i = 0; i < N; ++i)
    {
        mean += pts[i];
    }
    mean = mean/N;

    cv::Point2f mean_dev(0,0);

    for(int i = 0; i < N; ++i)
    {
        pts_norm[i] = pts[i] - mean;

        mean_dev.x += fabs(pts_norm[i].x);
        mean_dev.y += fabs(pts_norm[i].y);
    }
    mean_dev /= N;

    const float scale_x = 1.0/mean_dev.x;
    const float scale_y = 1.0/mean_dev.y;

    for(int i=0; i<N; i++)
    {
        pts_norm[i].x *= scale_x;
        pts_norm[i].y *= scale_y;
    }

    T <<  scale_x, 0, -mean.x*scale_x, 0, scale_y, -mean.y*scale_y, 0,0,1;
}

inline void Fundamental::computeErrors(const cv::Point2f& p1, const cv::Point2f& p2, const float* F, float& err1, float& err2)
{
    //! point X1 = (u1, v1, 1)^T in first image
    //! poInt X2 = (u2, v2, 1)^T in second image
    const float u1 = p1.x;
    const float v1 = p1.y;
    const float u2 = p2.x;
    const float v2 = p2.y;

    //! epipolar line in the second image L2 = (a2, b2, c2)^T = F   * X1
    const float a2 = F[0]*u1 + F[1]*v1 + F[2];
    const float b2 = F[3]*u1 + F[4]*v1 + F[5];
    const float c2 = F[6]*u1 + F[7]*v1 + F[8];
    //! epipolar line in the first image  L1 = (a1, b1, c1)^T = F^T * X2
    const float a1 = F[0]*u2 + F[3]*v2 + F[6];
    const float b1 = F[1]*u2 + F[4]*v2 + F[7];
    const float c1 = F[2]*u2 + F[5]*v2 + F[8];

    //! distance from point to line: d^2 = |ax+by+c|^2/(a^2+b^2)
    //! X2 to L2 in second image
    const float dist2 = a2*u2 + b2*v2 + c2;
    const float square_dist2 = dist2*dist2/(a2*a2 + b2*b2);
    //! X1 to L1 in first image
    const float dist1 = a1*u1 + b1*v1 + c1;
    const float square_dist1 = dist1*dist1/(a1*a1 + b1*b1);

    err1 = square_dist1;
    err2 = square_dist2;
}

//! modified from OpenCV
void Fundamental::decomposeEssentialMat(const cv::Mat& E, cv::Mat& R1, cv::Mat& R2, cv::Mat& t)
{
    assert(E.cols == 3 && E.rows == 3);

    cv::Mat U, D,Vt;
    cv::SVD::compute(E, D, U, Vt);

    if(determinant(U) < 0) U *= -1.;
    if(determinant(Vt) < 0) Vt *= -1.;

    cv::Mat W = (cv::Mat_<float>(3, 3) << 0, 1, 0, -1, 0, 0, 0, 0, 1);
    W.convertTo(W, E.type());

    R1 = U * W * Vt;
    R2 = U * W.t() * Vt;

    U.col(2).copyTo(t);
    t = t / cv::norm(t);
}

}