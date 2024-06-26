/*
    IEQ Pro driver

    Copyright (C) 2015 Jasem Mutlaq

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#pragma once

#include <string>
#include <vector>
#include <cstdint>

namespace iEQ
{

typedef enum { GPS_OFF, GPS_ON, GPS_DATA_OK } GPSStatus;
typedef enum
{
    ST_STOPPED,
    ST_TRACKING_PEC_OFF,
    ST_SLEWING,
    ST_GUIDING,
    ST_MERIDIAN_FLIPPING,
    ST_TRACKING_PEC_ON,
    ST_PARKED,
    ST_HOME
} SystemStatus;

typedef enum { TR_SIDEREAL, TR_LUNAR, TR_SOLAR, TR_KING, TR_CUSTOM } TrackRate;
typedef enum { SR_1, SR_2, SR_3, SR_4, SR_5, SR_6, SR_7, SR_8, SR_MAX } SlewRate;
typedef enum { TS_RS232, TS_CONTROLLER, TS_GPS } TimeSource;
typedef enum { HEMI_SOUTH, HEMI_NORTH } Hemisphere;
typedef enum { FW_MODEL, FW_BOARD, FW_CONTROLLER, FW_RA, FW_DEC } FirmwareItem;
typedef enum { RA_AXIS, DEC_AXIS } Axis;
typedef enum { IEQ_N, IEQ_S, IEQ_W, IEQ_E } Direction;
typedef enum { IEQ_SET_HOME, IEQ_GOTO_HOME, IEQ_FIND_HOME } HomeOperation;

typedef enum { IEQ_PIER_UNKNOWN = -1, IEQ_PIER_WEST = 0, IEQ_PIER_EAST = 1, IEQ_PIER_UNCERTAIN = 2 } IEQ_PIER_SIDE;

/**
 * @brief The BaseFirmware class provides control for iOptron version 2014 v2.0 protocol
 */
class Base
{
    public:

        Base();
        virtual ~Base() = default;

        typedef struct
        {
            GPSStatus gpsStatus;
            SystemStatus systemStatus;
            SystemStatus rememberSystemStatus;
            TrackRate trackRate;
            SlewRate slewRate;
            TimeSource timeSource;
            Hemisphere hemisphere;
            double longitude;
            double latitude;
        } Info;

        typedef struct
        {
            std::string Model;
            std::string MainBoardFirmware;
            std::string ControllerFirmware;
            std::string RAFirmware;
            std::string DEFirmware;
        } FirmwareInfo;

        typedef struct
        {
            std::string code;
            std::string model;
            std::string firmware;
        } MountInfo;

        void setDebugEnabled(bool enable)
        {
            m_IsDebug = enable;
        }
        void setDeviceName(const std::string &name)
        {
            m_DeviceName = name;
        }
        const char *getDeviceName()
        {
            return m_DeviceName.c_str();
        }

        /**************************************************************************
         Get Info
        **************************************************************************/
        /** Get iEQ current status info */
        bool getStatus(Info *info);
        /** Initializes communication with the mount and gets mount model */
        bool getModel();
        /** Get mainboard and controller firmware only */
        bool getMainFirmware();
        /** Get RA and DEC firmware info */
        bool getRADEFirmware();
        /** Get RA/DEC */
        bool getCoords(double *ra, double *dec);
        /** Get UTC/Date/Time */
        bool getUTCDateTime(double *utc_hours, int *yy, int *mm, int *dd, int *hh, int *minute, int *ss);
        /** Get firmware information */
        const FirmwareInfo &getFirmwareInfo() const
        {
            return m_FirmwareInfo;
        }

        bool getPierSide(IEQ_PIER_SIDE * pierSide);
    private:
        // read from mount using the GEA command
        double haAxis;      // not sure, try degrees to start with
        double decAxis;     // degrees, zero at 90 dec, sign determines pointing state
        double Ra;          // read from mount
        double Dec;
        Info info;
    public:

        /**************************************************************************
         Communication
        **************************************************************************/
        /**
        * @brief initCommunication Checks if communication with the mount is working
        * @param fd file descriptor
        * @param deviceName name of device used to log debugging messages.
        * @return True if communication is successful, false otherwise.
        */
        bool initCommunication(int fd);

        /**
         * @brief sendCommand Send a string command to device.
         * @param cmd Command to be sent. Can be either NULL TERMINATED or just byte buffer.
         * @param res If not nullptr, the function will wait for a response from the device. If nullptr, it returns true immediately
         * after the command is successfully sent.
         * @param cmd_len if -1, it is assumed that the @a cmd is a null-terminated string. Otherwise, it would write @a cmd_len bytes from
         * the @a cmd buffer.
         * @param res_len if -1 and if @a res is not nullptr, the function will read until it detects the default delimiter DRIVER_STOP_CHAR
         *  up to DRIVER_LEN length. Otherwise, the function will read @a res_len from the device and store it in @a res.
         * @return True if successful, false otherwise.
         */
        bool sendCommand(const char * cmd, char * res = nullptr, int cmd_len = -1, int res_len = -1);

        /**
         * @brief hexDump Helper function to print non-string commands to the logger so it is easier to debug
         * @param buf buffer to format the command into hex strings.
         * @param data the command
         * @param size length of the command in bytes.
         * @note This is called internally by sendCommand, no need to call it directly.
         */
        void hexDump(char * buf, const char * data, int size);

        /**
         * @brief isCommandSupported Check if specific iOptron command is supported for this mount model
         * @param command command code (e.g. MS)
         * @param silent if false (default), it will report why command is not supported. If true, it will not print any messages.
         * @return True if supported, false otherwise
         */
        bool isCommandSupported(const std::string &command, bool silent = false);


        /**************************************************************************
         Motion
        **************************************************************************/
        virtual bool startMotion(Direction dir);
        virtual bool stopMotion(Direction dir);
        virtual bool setSlewRate(SlewRate rate);
        virtual bool setCustomRATrackRate(double rate);
        virtual bool setTrackMode(TrackRate rate);
        virtual bool setTrackEnabled(bool enabled);
        /* v3.0 Add in PEC Control */
        bool setPECEnabled(bool enabled); // start / stop PEC
        bool setPETEnabled(bool enabled); // record / cancel PEC
        bool getPETEnabled(bool enabled); // not supported on current firmware
        // End Mod */
        virtual bool abort();
        virtual bool slew();
        virtual bool sync();
        virtual bool setRA(double ra);
        virtual bool setDE(double dec);
        virtual bool setAz(double az);
        virtual bool setAlt(double alt);

        /**************************************************************************
         Home
        **************************************************************************/
        virtual bool findHome();
        virtual bool gotoHome();
        virtual bool setCurrentHome();

        /**************************************************************************
         Park
        **************************************************************************/
        virtual bool park();
        virtual bool unpark();
        virtual bool setParkAz(double az);
        virtual bool setParkAlt(double alt);

        /**************************************************************************
         Guide
        **************************************************************************/
        virtual bool setGuideRate(double raRate, double deRate);
        virtual bool getGuideRate(double * raRate, double * deRate);
        virtual bool startGuide(Direction dir, uint32_t ms);

        /**************************************************************************
         Time & Location
        **************************************************************************/
        virtual bool setLongitude(double longitude);
        virtual bool setLatitude(double latitude);
        virtual bool setLocalDate(int yy, int mm, int dd);
        virtual bool setLocalTime(int hh, int mm, int ss);
        virtual bool setUTCOffset(double offset_hours);
        virtual bool setDST(bool enabled);

        /**************************************************************************
         Simulation
        **************************************************************************/
        void setSimGPSstatus(GPSStatus value);
        void setSimSytemStatus(SystemStatus value);
        void setSimTrackRate(TrackRate value);
        void setSimSlewRate(SlewRate value);
        void setSimTimeSource(TimeSource value);
        void setSimHemisphere(Hemisphere value);
        void setSimRA(double ra);
        void setSimDE(double de);
        void setSimLongLat(double longitude, double latitude);
        void setSimGuideRate(double raRate, double deRate);
        void setSimulation(bool enable);

    protected:

        int m_PortFD {-1};
        std::string m_DeviceName { "iEQ" };
        bool m_IsDebug { false };
        bool m_Simulation = {false};
        static const uint8_t DRIVER_TIMEOUT { 3 };
        static const uint8_t DRIVER_LEN { 64 };
        static const char DRIVER_STOP_CHAR { '#' };

        FirmwareInfo m_FirmwareInfo;

        const std::vector<MountInfo> m_MountList =
        {
            {"0010", "Cube II EQ", "160610"},
            {"0011", "Smart EQ Pro+", "161028"},
            {"0025", "CEM25", "170106"},
            {"0026", "CEM25-EC", "170518"},
            {"0030", "iEQ30 Pro", "161101"},
            {"0040", "CEM40", "181018"},
            {"0041", "CEM40-EC", "181018"},
            {"0043", "GEM45", "191018"},
            {"0044", "GEM45-EC", "191018"},
            {"0045", "iEQ45 Pro EQ", "161101"},
            {"0046", "iEQ45 Pro AA", "161101"},
            {"0060", "CEM60", "161101"},
            {"0061", "CEM60-EC", "161101"},
            {"5010", "Cube II AA", "160610"},
            {"5035", "AZ Mount Pro", "170410"},
        };

        struct
        {
            double ra;
            double de;
            double ra_guide_rate;
            double de_guide_rate;
            double JD;
            int utc_offset_minutes;
            bool day_light_saving;
            IEQ_PIER_SIDE pier_state;

            Info simInfo;
        } simData;

        ///
        /// \brief DecodeString converts the string to a double by dividing by the factor
        /// \param data
        /// \param size
        /// \param factor
        /// \return
        ///
        double DecodeString(const char * data, size_t size, double factor);

        ///
        /// \brief DecodeString converts a string of defined size to an int
        /// \param data
        /// \param size
        /// \return
        ///
        int DecodeString(const char * data, size_t size);

        constexpr static const double ieqDegrees { 60.0 * 60.0 * 100.0 };
        constexpr static const double ieqHours { 60.0 * 60.0 * 1000.0 };
};

class Simulator: public Base
{
        Simulator();
        virtual ~Simulator();


};

}
