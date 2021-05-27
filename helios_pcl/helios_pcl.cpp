/*******
***
***Lucid Helios -> PCL
**************/

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>

typedef pcl::PointXYZ pclType;
typedef pcl::PointCloud<pclType> pclCloud;

//#include "stdafx.h"
#include "ArenaApi.h"
#include "SaveApi.h"

#define TAB1 "  "
#define TAB2 "    "
#define TAB3 "      "

// Helios: Min/Max Depth
//    This example captures a 3D image and interprets the ABCY data into their
//    appropriate x, y and z coordinates and intensities. It converts this data
//    into millimeters and then displays this data for points with both the
//    largest and smallest values of z.

// =-=-=-=-=-=-=-=-=-
// =-=- SETTINGS =-=-
// =-=-=-=-=-=-=-=-=-

// file name
#define FILE_NAME "Images/Cpp_Helios_MinMaxDepth.ply"

// pixel format
#define PIXEL_FORMAT "Coord3D_ABCY16"

// image timeout
#define IMAGE_TIMEOUT 2000


template<typename T>
T Points_to_PCLType(double& x, double& y, double& z)
{
    T outType;
    outType.x = x;
    outType.y = y;
    outType.z = z;

    return outType;
}


// =-=-=-=-=-=-=-=-=-
// =-=- EXAMPLE -=-=-
// =-=-=-=-=-=-=-=-=-

// store x, y, z data in mm and intensity for a given point
struct PointData
{
    int16_t x;
    int16_t y;
    int16_t z;
    int16_t intensity;
};





// demonstrates acquiring 3D data for a specific point
// (1) gets image
// (2) interprets ABCY data to get x, y, z and intensity
// (3) stores data for point with min and max z values
// (4) displays 3D data for min and max points
pclCloud::Ptr getPCLCloud(Arena::IDevice* pDevice)
{
    GenApi::INodeMap* pNodeMap = pDevice->GetNodeMap();

    // validate if Scan3dCoordinateSelector node exists. If not - probaly not
    // Helios camera used running the example
    GenApi::CEnumerationPtr checkpCoordSelector = pNodeMap->GetNode("Scan3dCoordinateSelector");
    if (!checkpCoordSelector)
    {
        std::cout << TAB1 << "Scan3dCoordinateSelector node is not found. Please make sure that Helios device is used for the example.\n";
        return nullptr;
    }

    // validate if Scan3dCoordinateOffset node exists. If not - probaly Helios
    // has an old firmware
    GenApi::CFloatPtr checkpCoord = pNodeMap->GetNode("Scan3dCoordinateOffset");
    if (!checkpCoord)
    {
        std::cout << TAB1 << "Scan3dCoordinateOffset node is not found. Please update Helios firmware.\n";
        return nullptr;
    }

    // check if Helios2 camera used for the example
    bool isHelios2 = false;
    GenICam::gcstring deviceModelName = Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "DeviceModelName");
    std::string deviceModelName_tmp = deviceModelName.c_str();
    if (deviceModelName_tmp.rfind("HLT", 0) == 0)
    {
        isHelios2 = true;
    }

    // get node values that will be changed in order to return their values at the end of the example
    GenICam::gcstring pixelFormatInitial = Arena::GetNodeValue<GenICam::gcstring>(pNodeMap, "PixelFormat");
    GenICam::gcstring operatingModeInitial = Arena::GetNodeValue<GenICam::gcstring>(pNodeMap, "Scan3dOperatingMode");

    // Set pixel format
    // Warning: HLT003S-001 / Helios2 - has only Coord3D_ABCY16 in this case
    //    This example demonstrates data interpretation for both a signed or
    //    unsigned pixel format. Default PIXEL_FORMAT here is set to
    //    Coord3D_ABCY16 but this can be modified to be a signed pixel format
    //    by changing it to Coord3D_ABCY16s.
    std::cout << TAB1 << "Set " << PIXEL_FORMAT << " to pixel format\n";

    Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "PixelFormat", PIXEL_FORMAT);

    // set operating mode distance
    if (isHelios2)
    {
        std::cout << TAB1 << "Set 3D operating mode to Distance5000mm\n";
        Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "Scan3dOperatingMode", "Distance5000mmMultiFreq");
    }
    else
    {
        std::cout << TAB1 << "Set 3D operating mode to Distance1500mm\n";
        Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "Scan3dOperatingMode", "Distance1500mm");
    }

    // get the coordinate scale in order to convert x, y and z values to mm as
    // well as the offset for x and y to correctly adjust values when in an
    // unsigned pixel format
    std::cout << TAB1 << "Get xyz coordinate scales and offsets\n\n";

    Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "Scan3dCoordinateSelector", "CoordinateA");
    // getting scaleX as float by casting since SetPly() will expect it passed as
    // float
    double scaleX = static_cast<double>(Arena::GetNodeValue<double>(pNodeMap, "Scan3dCoordinateScale"));
    // getting offsetX as float by casting since SetPly() will expect it passed
    // as float
    float offsetX = static_cast<float>(Arena::GetNodeValue<double>(pNodeMap, "Scan3dCoordinateOffset"));
    Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "Scan3dCoordinateSelector", "CoordinateB");
    double scaleY = Arena::GetNodeValue<double>(pNodeMap, "Scan3dCoordinateScale");
    // getting offsetY as float by casting since SetPly() will expect it passed
    // as float
    float offsetY = static_cast<float>(Arena::GetNodeValue<double>(pNodeMap, "Scan3dCoordinateOffset"));
    Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "Scan3dCoordinateSelector", "CoordinateC");
    double scaleZ = Arena::GetNodeValue<double>(pNodeMap, "Scan3dCoordinateScale");

    // enable stream auto negotiate packet size
    Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", true);

    // enable stream packet resend
    Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamPacketResendEnable", true);

    // retrieve image
    std::cout << TAB2 << "Acquire image\n";

    pDevice->StartStream();
    Arena::IImage* pImage = pDevice->GetImage(IMAGE_TIMEOUT);

    // prepare info from input buffer
    size_t width = pImage->GetWidth();
    size_t height = pImage->GetHeight();
    size_t size = width * height;
    size_t srcBpp = pImage->GetBitsPerPixel();
    size_t srcPixelSize = srcBpp / 8;
    const uint8_t* pInput = pImage->GetData();
    const uint8_t* pIn = pInput;

    // using strcmp to avoid conversion issue
    int compareResult_ABCY16s = strcmp(PIXEL_FORMAT, "Coord3D_ABCY16s"); // if they are equal compareResult_ABCY16s = 0
    int compareResult_ABCY16 = strcmp(PIXEL_FORMAT, "Coord3D_ABCY16");       // if they are equal compareResult_ABCY16 = 0

    bool isSignedPixelFormat = false;

    pclCloud::Ptr outCloud(new pclCloud);
    outCloud->resize(size);
    outCloud->width = width;
    outCloud->height = height;
    // if PIXEL_FORMAT is equal to Coord3D_ABCY16s
    if (compareResult_ABCY16s == 0)
    {
        isSignedPixelFormat = true;

        for (size_t i = 1; i < size; i++)
        {
            auto pIndex = pIn + srcPixelSize * i;
            // Extract point data to signed 16 bit integer
            //    The first channel is the x coordinate, second channel is the y
            //    coordinate, the third channel is the z coordinate and the
            //    fourth channel is intensity. We offset pIn by 2 for each
            //    channel because pIn is an 8 bit integer and we want to read it
            //    as a 16 bit integer.
            int16_t x = *reinterpret_cast<const int16_t*>(pIndex);
            int16_t y = *reinterpret_cast<const int16_t*>((pIndex + 2));
            int16_t z = *reinterpret_cast<const int16_t*>((pIndex + 4));
            int16_t intensity = *reinterpret_cast<const int16_t*>((pIndex + 6));

            // convert x, y and z values to mm using their coordinate scales
            double x_ = double(x) * scaleX;
            double y_ = double(y) * scaleY;
            double z_ = double(z) * scaleZ;

            auto tempP = Points_to_PCLType<pclType>(x_, y_, z_);
            outCloud->at(i) = tempP;

            //pIn += srcPixelSize;
        }
    }
    // if PIXEL_FORMAT is equal to Coord3D_ABCY16
    else if (compareResult_ABCY16 == 0)
    {
        for (size_t i = 1; i < size; i++)
        {
            auto pIndex = pIn + srcPixelSize * i;
            // Extract point data to signed 16 bit integer
            //    The first channel is the x coordinate, second channel is the y
            //    coordinate, the third channel is the z coordinate and the
            //    fourth channel is intensity. We offset pIn by 2 for each
            //    channel because pIn is an 8 bit integer and we want to read it
            //    as a 16 bit integer.
            uint16_t x = *reinterpret_cast<const uint16_t*>(pIndex);
            uint16_t y = *reinterpret_cast<const uint16_t*>((pIndex + 2));
            uint16_t z = *reinterpret_cast<const uint16_t*>((pIndex + 4));
            uint16_t intensity = *reinterpret_cast<const uint16_t*>((pIndex + 6));

            // if z is less than max value, as invalid values get filtered to
            // 65535
            if (z < 65535)
            {
                // Convert x, y and z to millimeters
                //    Using each coordinates' appropriate scales, convert x, y
                //    and z values to mm. For the x and y coordinates in an
                //    unsigned pixel format, we must then add the offset to our
                //    converted values in order to get the correct position in
                //    millimeters.
                double x_ = double(x) * scaleX + offsetX;
                double y_ = (double(y) * scaleY) + offsetY;
                double z_ = (double(z) * scaleZ);

                auto tempP = Points_to_PCLType<pclType>(x_, y_, z_);
                outCloud->at(i) = tempP;
            }

            //pIn += srcPixelSize;
        }

    }
    else
    {
        std::cout << "This example requires the camera to be in either 3D image format Coord3D_ABCY16 or Coord3D_ABCY16s\n\n";
    }

    // clean up
    pDevice->RequeueBuffer(pImage);
    pDevice->StopStream();

    // return nodes to their initial values
    Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "Scan3dOperatingMode", operatingModeInitial);
    Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "PixelFormat", pixelFormatInitial);
    std::cout << TAB1 << "Nodes were set back to initial values\n";

    return outCloud;
}

// =-=-=-=-=-=-=-=-=-
// =- PREPARATION -=-
// =- & CLEAN UP =-=-
// =-=-=-=-=-=-=-=-=-

int main()
{
    // flag to track when an exception has been thrown
    bool exceptionThrown = false;

    std::cout << "Cpp_Helios_PCL\n";

    try
    {
        // prepare example
        Arena::ISystem* pSystem = Arena::OpenSystem();
        pSystem->UpdateDevices(100);
        std::vector<Arena::DeviceInfo> deviceInfos = pSystem->GetDevices();
        if (deviceInfos.size() == 0)
        {
            std::cout << "\nNo camera connected\nPress enter to complete\n";
            std::getchar();
            return 0;
        }
        Arena::IDevice* pDevice = pSystem->CreateDevice(deviceInfos[0]);

        std::cout << "Commence example\n\n";

        // run example
        auto pcldata = getPCLCloud(pDevice);

        std::cout << "Save PCL PLY" << std::endl;
        pcl::io::savePLYFile("pclPLY.ply", *pcldata);

        std::cout << "\nExample complete\n";

        // clean up example
        pSystem->DestroyDevice(pDevice);
        Arena::CloseSystem(pSystem);
    }
    catch (GenICam::GenericException& ge)
    {
        std::cout << "\nGenICam exception thrown: " << ge.what() << "\n";
        exceptionThrown = true;
    }
    catch (std::exception& ex)
    {
        std::cout << "\nStandard exception thrown: " << ex.what() << "\n";
        exceptionThrown = true;
    }
    catch (...)
    {
        std::cout << "\nUnexpected exception thrown\n";
        exceptionThrown = true;
    }

    std::cout << "Press enter to complete\n";
    std::getchar();

    if (exceptionThrown)
        return -1;
    else
        return 0;
}