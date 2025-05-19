#pragma once

#include <vector>
#include <cstdint>
#include <unordered_map>
#include <array>
#include <ctime> // for time_t

namespace XinTan
{

#define DEPTH_ABNORMAL 964000
#define DEPTH_LOW 964001
#define DEPTH_IGNORE 964008
#define AMPLITUDE_ABNORMAL 64000
#define AMPLITUDE_LOW 64001
#define AMPLITUDE_ADC 64002
#define AMPLITUDE_OVEREXPOSURE 64003
#define AMPLITUDE_DEFECTIVE 64004
#define AMPLITUDE_INVALID 64007

typedef void (*LogFunction)(const char *pfunction, const char *plogstr);

enum DataType
{
    AMPLITUDE,
    DISTANCE,
    RESERVER,
    GRAYSCALE
};

const std::unordered_map<uint8_t, uint8_t> devsdk_feq_map = {{0, 2 },
                                                             {1, 0 },
                                                             {3, 1 },
                                                             {4, 6 },
                                                             {7, 3 },
                                                             {15, 4 }};

const std::array<float, 7> refcof_S240MINI ={0.051779f, //FREQ_12M
                                              0.038790f, //FREQ_6M
                                              0.070588f, //FREQ_24M
                                              0.051976f, //FREQ_3M
                                              0.041449f, //FREQ_1_5M
                                              0.0399f, //FREQ_0_75M
                                              0.047257f}; //FREQ_4_8M


const std::array<float, 7> refcof_S240PRO ={0.027421f, //FREQ_12M
                                             0.020683f, //FREQ_6M
                                             0.039181f, //FREQ_24M
                                             0.029102f, //FREQ_3M
                                             0.021447f, //FREQ_1_5M
                                             0.0198f, //FREQ_0_75M
                                             0.024667f}; //FREQ_4_8M

const std::array<float, 7> refcof_M240MIN ={0.008084f * 2.0f, //FREQ_12M
                                             0.006265f * 2.0f, //FREQ_6M
                                             0.010401f * 2.0f, //FREQ_24M
                                             0.006837f * 2.0f, //FREQ_3M
                                             0.005828f * 2.0f, //FREQ_1_5M
                                             0.00795f * 2.0f, //FREQ_0_75M
                                             0.005828f * 2.0f}; //FREQ_4_8M

const std::array<float, 7> refcof_M240PRO ={0.004703f * 2.0f, //FREQ_12M
                                             0.004804f * 2.0f, //FREQ_6M
                                             0.006233f * 2.0f, //FREQ_24M
                                             0.004220f * 2.0f, //FREQ_3M
                                             0.003947f * 2.0f, //FREQ_1_5M
                                             0.00436f * 2.0f, //FREQ_0_75M
                                             0.004463f * 2.0f}; //FREQ_4_8M

const std::array<float, 7> refcof_M240MAX ={0.002572f, //FREQ_12M
                                             0.002509f, //FREQ_6M
                                             0.004244f, //FREQ_24M
                                             0.002066f, //FREQ_3M
                                             0.002140f, //FREQ_1_5M
                                             0.00104348f, //FREQ_0_75M
                                             0.002406f}; //FREQ_4_8M

const std::array<float, 7> refcof_M240ULTRA ={0.002282f * 4.0f, //FREQ_12M
                                               0.001671f * 4.0f, //FREQ_6M
                                               0.002710f * 4.0f, //FREQ_24M
                                               0.002112f * 4.0f, //FREQ_3M
                                               0.001516f * 4.0f, //FREQ_1_5M
                                               0.0019f * 4.0f, //FREQ_0_75M
                                               0.001614f * 4.0f}; //FREQ_4_8M

const std::array<float, 7> refcof_M60 ={0.029626f, //FREQ_12M
                                         0.027954f, //FREQ_6M
                                         0.047552f, //FREQ_24M
                                         0.024505f, //FREQ_3M
                                         0.023817f, //FREQ_1_5M
                                         0.035275f, //FREQ_0_75M
                                         0.030925f}; //FREQ_4_8M




typedef std::vector<uint8_t> XByteArray;

struct XtPointXYZI {
    float x;
    float y;
    float z;
    float intensity;
};

struct CamParameterS
{
    float fx;
    float fy;
    float cx;
    float cy;
    float k1;
    float k2;
    float k3;
    float p1;
    float p2;
};


enum IMAGE_FLAG {
    IMG_DIST = 0x01,
    IMG_AMP = 0x02,
    IMG_LEVEL = 0x04,
    IMG_GS16 = 0x08,
    IMG_DCS = 0x10,
    IMG_GS8 = 0x20,
    IMG_DCSN = 0x40
};

class IFrame
{
public:
    virtual ~IFrame() = default;
    enum DataType
    {
        AMPLITUDE,
        DISTANCE,
        RESERVER,
        GRAYSCALE
    };
    // 获取数据
    virtual const int getDistDataSize() = 0;
    virtual const uint32_t getDistData(const size_t &index) = 0;
    virtual const uint16_t getAmplData(const size_t &index) = 0;
    virtual const float getMaxAmplData() = 0;
    virtual const uint32_t getRawDistData(const size_t &index) = 0;
    virtual const uint16_t getGrayscaleData(const size_t &index) = 0;
    virtual const float getReflectivity(const size_t &index) = 0;
    virtual const uint8_t getleveldata(const size_t &index) = 0;
    virtual const uint8_t getMotionFlag(const size_t &index) = 0;
    virtual const uint8_t getFreqMap(const size_t &index) = 0;
    virtual const uint16_t getIntMap(const size_t &index) = 0;

    virtual uint32_t* getDistDataBuffer() = 0;
    virtual uint16_t* getAmplDataBuffer() = 0;
    virtual uint32_t* getRawDistDataBuffer() = 0;
    virtual uint16_t* getGrayscaleDataBuffer() = 0;
    virtual float* getReflectivityBuffer() = 0;
    virtual uint8_t* getleveldataBuffer() = 0;
    virtual uint8_t* getMotionFlagBuffer() = 0;
    virtual uint8_t* getFreqMapBuffer() = 0;
    virtual uint16_t* getIntMapBuffer() = 0;


    // virtual const std::vector<uint8_t>& getFrameData() = 0;
    // virtual  std::vector<uint16_t>* getDistData() = 0;
    // virtual const std::vector<uint16_t>& getAmplData() = 0;
    // virtual const std::vector<uint16_t>& getReflectivity() = 0;
    // virtual const std::vector<XtPointXYZI>& getPoints() = 0;
    // virtual const std::vector<uint16_t>& getRawDistData() = 0;
    // virtual const time_t* getTimeAlg() = 0;
    //  纯虚函数声明
    virtual uint8_t getFrameVersion() = 0;
    virtual uint16_t getPixelDataOffset() = 0;
    virtual uint64_t getFrameId() = 0;
    virtual uint16_t getDataType() = 0;
    virtual uint16_t getWidth() = 0;
    virtual uint16_t getHeight() = 0;
    virtual uint32_t getPxSize() = 0;
    virtual uint16_t getRoiX0() = 0;
    virtual uint16_t getRoiY0() = 0;
    virtual uint8_t getBinning() = 0;
    virtual uint64_t getTimeStampS() = 0;
    virtual uint32_t getTimeStampNS() = 0;
    virtual uint8_t getTimeStampState() = 0;
    virtual uint8_t getTimeStampType() = 0;
    virtual int16_t getTemperature() = 0;
    virtual int16_t getVcselTemperature() = 0;
    virtual int getDustPercent() = 0;
    virtual bool getNeedXiaCaiYang() = 0;
    virtual uint16_t getXBinning() = 0;
    virtual uint16_t getYBinning() = 0;
    virtual uint16_t getOrgWidth() = 0;
    virtual uint16_t getOrgHeight() = 0;
    virtual bool getHasPointCloud() = 0;

    // 设置属性的方法可以根据需要添加，这里只添加几个示例
    virtual void setFrameId(uint64_t id) = 0;
    virtual void setTimeStampS(uint64_t ts) = 0;
    virtual void setDistdataIndex(const size_t &index, const uint32_t &data) = 0;
    virtual void setRefdataIndex(const size_t &index, const float &data) = 0;
    virtual void setAmpdataIndex(const size_t &index, const uint16_t &data) = 0;
    virtual void setMotionIndex(const size_t &index, const uint8_t &data) = 0;
    virtual void setDustPercent(int data) = 0;
    // 其他方法
    virtual void sortData(const XByteArray &xByteArray) = 0;
};

} // namespace XinTan
