//
// This file is auto-generated. Please don't modify it!
//
package org.opencv.video;

import org.opencv.core.Size;
import org.opencv.core.TermCriteria;

// C++: class SparsePyrLKOpticalFlow
//javadoc: SparsePyrLKOpticalFlow

public class SparsePyrLKOpticalFlow extends SparseOpticalFlow {

    protected SparsePyrLKOpticalFlow(long addr) {
        super(addr);
    }

    // internal usage only
    public static SparsePyrLKOpticalFlow __fromPtr__(long addr) {
        return new SparsePyrLKOpticalFlow(addr);
    }

    //
    // C++: static Ptr_SparsePyrLKOpticalFlow cv::SparsePyrLKOpticalFlow::create(Size winSize = Size(21, 21), int maxLevel = 3, TermCriteria crit = TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01), int flags = 0, double minEigThreshold = 1e-4)
    //

    //javadoc: SparsePyrLKOpticalFlow::create(winSize, maxLevel, crit, flags, minEigThreshold)
    public static SparsePyrLKOpticalFlow create(Size winSize, int maxLevel, TermCriteria crit, int flags, double minEigThreshold) {

        SparsePyrLKOpticalFlow retVal = SparsePyrLKOpticalFlow.__fromPtr__(create_0(winSize.width, winSize.height, maxLevel, crit.type, crit.maxCount, crit.epsilon, flags, minEigThreshold));

        return retVal;
    }

    //javadoc: SparsePyrLKOpticalFlow::create(winSize, maxLevel, crit, flags)
    public static SparsePyrLKOpticalFlow create(Size winSize, int maxLevel, TermCriteria crit, int flags) {

        SparsePyrLKOpticalFlow retVal = SparsePyrLKOpticalFlow.__fromPtr__(create_1(winSize.width, winSize.height, maxLevel, crit.type, crit.maxCount, crit.epsilon, flags));

        return retVal;
    }

    //javadoc: SparsePyrLKOpticalFlow::create(winSize, maxLevel, crit)
    public static SparsePyrLKOpticalFlow create(Size winSize, int maxLevel, TermCriteria crit) {

        SparsePyrLKOpticalFlow retVal = SparsePyrLKOpticalFlow.__fromPtr__(create_2(winSize.width, winSize.height, maxLevel, crit.type, crit.maxCount, crit.epsilon));

        return retVal;
    }

    //javadoc: SparsePyrLKOpticalFlow::create(winSize, maxLevel)
    public static SparsePyrLKOpticalFlow create(Size winSize, int maxLevel) {

        SparsePyrLKOpticalFlow retVal = SparsePyrLKOpticalFlow.__fromPtr__(create_3(winSize.width, winSize.height, maxLevel));

        return retVal;
    }

    //javadoc: SparsePyrLKOpticalFlow::create(winSize)
    public static SparsePyrLKOpticalFlow create(Size winSize) {

        SparsePyrLKOpticalFlow retVal = SparsePyrLKOpticalFlow.__fromPtr__(create_4(winSize.width, winSize.height));

        return retVal;
    }

    //javadoc: SparsePyrLKOpticalFlow::create()
    public static SparsePyrLKOpticalFlow create() {

        SparsePyrLKOpticalFlow retVal = SparsePyrLKOpticalFlow.__fromPtr__(create_5());

        return retVal;
    }


    //
    // C++:  Size cv::SparsePyrLKOpticalFlow::getWinSize()
    //

    // C++: static Ptr_SparsePyrLKOpticalFlow cv::SparsePyrLKOpticalFlow::create(Size winSize = Size(21, 21), int maxLevel = 3, TermCriteria crit = TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01), int flags = 0, double minEigThreshold = 1e-4)
    private static native long create_0(double winSize_width, double winSize_height, int maxLevel, int crit_type, int crit_maxCount, double crit_epsilon, int flags, double minEigThreshold);


    //
    // C++:  TermCriteria cv::SparsePyrLKOpticalFlow::getTermCriteria()
    //

    private static native long create_1(double winSize_width, double winSize_height, int maxLevel, int crit_type, int crit_maxCount, double crit_epsilon, int flags);


    //
    // C++:  double cv::SparsePyrLKOpticalFlow::getMinEigThreshold()
    //

    private static native long create_2(double winSize_width, double winSize_height, int maxLevel, int crit_type, int crit_maxCount, double crit_epsilon);


    //
    // C++:  int cv::SparsePyrLKOpticalFlow::getFlags()
    //

    private static native long create_3(double winSize_width, double winSize_height, int maxLevel);


    //
    // C++:  int cv::SparsePyrLKOpticalFlow::getMaxLevel()
    //

    private static native long create_4(double winSize_width, double winSize_height);


    //
    // C++:  void cv::SparsePyrLKOpticalFlow::setFlags(int flags)
    //

    private static native long create_5();


    //
    // C++:  void cv::SparsePyrLKOpticalFlow::setMaxLevel(int maxLevel)
    //

    // C++:  Size cv::SparsePyrLKOpticalFlow::getWinSize()
    private static native double[] getWinSize_0(long nativeObj);


    //
    // C++:  void cv::SparsePyrLKOpticalFlow::setMinEigThreshold(double minEigThreshold)
    //

    // C++:  TermCriteria cv::SparsePyrLKOpticalFlow::getTermCriteria()
    private static native double[] getTermCriteria_0(long nativeObj);


    //
    // C++:  void cv::SparsePyrLKOpticalFlow::setTermCriteria(TermCriteria crit)
    //

    // C++:  double cv::SparsePyrLKOpticalFlow::getMinEigThreshold()
    private static native double getMinEigThreshold_0(long nativeObj);


    //
    // C++:  void cv::SparsePyrLKOpticalFlow::setWinSize(Size winSize)
    //

    // C++:  int cv::SparsePyrLKOpticalFlow::getFlags()
    private static native int getFlags_0(long nativeObj);

    // C++:  int cv::SparsePyrLKOpticalFlow::getMaxLevel()
    private static native int getMaxLevel_0(long nativeObj);

    // C++:  void cv::SparsePyrLKOpticalFlow::setFlags(int flags)
    private static native void setFlags_0(long nativeObj, int flags);

    // C++:  void cv::SparsePyrLKOpticalFlow::setMaxLevel(int maxLevel)
    private static native void setMaxLevel_0(long nativeObj, int maxLevel);

    // C++:  void cv::SparsePyrLKOpticalFlow::setMinEigThreshold(double minEigThreshold)
    private static native void setMinEigThreshold_0(long nativeObj, double minEigThreshold);

    // C++:  void cv::SparsePyrLKOpticalFlow::setTermCriteria(TermCriteria crit)
    private static native void setTermCriteria_0(long nativeObj, int crit_type, int crit_maxCount, double crit_epsilon);

    // C++:  void cv::SparsePyrLKOpticalFlow::setWinSize(Size winSize)
    private static native void setWinSize_0(long nativeObj, double winSize_width, double winSize_height);

    // native support for java finalize()
    private static native void delete(long nativeObj);

    //javadoc: SparsePyrLKOpticalFlow::getWinSize()
    public Size getWinSize() {

        Size retVal = new Size(getWinSize_0(nativeObj));

        return retVal;
    }

    //javadoc: SparsePyrLKOpticalFlow::setWinSize(winSize)
    public void setWinSize(Size winSize) {

        setWinSize_0(nativeObj, winSize.width, winSize.height);

        return;
    }

    //javadoc: SparsePyrLKOpticalFlow::getTermCriteria()
    public TermCriteria getTermCriteria() {

        TermCriteria retVal = new TermCriteria(getTermCriteria_0(nativeObj));

        return retVal;
    }

    //javadoc: SparsePyrLKOpticalFlow::setTermCriteria(crit)
    public void setTermCriteria(TermCriteria crit) {

        setTermCriteria_0(nativeObj, crit.type, crit.maxCount, crit.epsilon);

        return;
    }

    //javadoc: SparsePyrLKOpticalFlow::getMinEigThreshold()
    public double getMinEigThreshold() {

        double retVal = getMinEigThreshold_0(nativeObj);

        return retVal;
    }

    //javadoc: SparsePyrLKOpticalFlow::setMinEigThreshold(minEigThreshold)
    public void setMinEigThreshold(double minEigThreshold) {

        setMinEigThreshold_0(nativeObj, minEigThreshold);

        return;
    }

    //javadoc: SparsePyrLKOpticalFlow::getFlags()
    public int getFlags() {

        int retVal = getFlags_0(nativeObj);

        return retVal;
    }

    //javadoc: SparsePyrLKOpticalFlow::setFlags(flags)
    public void setFlags(int flags) {

        setFlags_0(nativeObj, flags);

        return;
    }

    //javadoc: SparsePyrLKOpticalFlow::getMaxLevel()
    public int getMaxLevel() {

        int retVal = getMaxLevel_0(nativeObj);

        return retVal;
    }

    //javadoc: SparsePyrLKOpticalFlow::setMaxLevel(maxLevel)
    public void setMaxLevel(int maxLevel) {

        setMaxLevel_0(nativeObj, maxLevel);

        return;
    }

    @Override
    protected void finalize() throws Throwable {
        delete(nativeObj);
    }

}
