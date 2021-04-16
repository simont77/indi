/*
    INDI IEQ Pro driver

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

#include "ieqpro.h"
#include "indicom.h"
#include "connectionplugins/connectionserial.h"

#include <libnova/sidereal_time.h>
#include <libnova/transform.h>
#include <memory>
#include <thread>
#include <chrono>

#include <cmath>
#include <cstring>

/* Simulation Parameters */
#define SLEWRATE 1          /* slew rate, degrees/s */

#define MOUNTINFO_TAB "Mount Info"
#define PEC_TAB "PEC training"  

using namespace iEQ;

// We declare an auto pointer to IEQPro.
static std::unique_ptr<IEQPro> scope(new IEQPro());

void ISGetProperties(const char *dev)
{
    scope->ISGetProperties(dev);
}

void ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    scope->ISNewSwitch(dev, name, states, names, n);
}

void ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    scope->ISNewText(dev, name, texts, names, n);
}

void ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    scope->ISNewNumber(dev, name, values, names, n);
}

void ISNewBLOB(const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[],
               char *names[], int n)
{
    INDI_UNUSED(dev);
    INDI_UNUSED(name);
    INDI_UNUSED(sizes);
    INDI_UNUSED(blobsizes);
    INDI_UNUSED(blobs);
    INDI_UNUSED(formats);
    INDI_UNUSED(names);
    INDI_UNUSED(n);
}
void ISSnoopDevice(XMLEle *root)
{
    scope->ISSnoopDevice(root);
}

IEQPro::IEQPro()
{
    setVersion(1, 9);

    driver.reset(new Base());

    scopeInfo.gpsStatus    = GPS_OFF;
    scopeInfo.systemStatus = ST_STOPPED;
    scopeInfo.trackRate    = TR_SIDEREAL;
    /* v3.0 use default PEC Settings */
    scopeInfo.systemStatus = ST_TRACKING_PEC_OFF;
    // End Mod */
    scopeInfo.slewRate     = SR_1;
    scopeInfo.timeSource   = TS_RS232;
    scopeInfo.hemisphere   = HEMI_NORTH;

    DBG_SCOPE = INDI::Logger::getInstance().addDebugLevel("Scope Verbose", "SCOPE");

    SetTelescopeCapability(TELESCOPE_CAN_PARK | TELESCOPE_CAN_SYNC | TELESCOPE_CAN_GOTO | TELESCOPE_CAN_ABORT |
                            /* v3.0 use default PEC Settings */
                           TELESCOPE_HAS_PEC  |
                           // End Mod */
                           TELESCOPE_HAS_TIME | TELESCOPE_HAS_LOCATION | TELESCOPE_HAS_TRACK_MODE | TELESCOPE_CAN_CONTROL_TRACK |
                           TELESCOPE_HAS_TRACK_RATE,
                           9);
}

const char *IEQPro::getDefaultName()
{
    return "iEQ";
}

bool IEQPro::initProperties()
{
    INDI::Telescope::initProperties();

    /* Firmware */
    IUFillText(&FirmwareT[FW_MODEL], "Model", "", nullptr);
    IUFillText(&FirmwareT[FW_BOARD], "Board", "", nullptr);
    IUFillText(&FirmwareT[FW_CONTROLLER], "Controller", "", nullptr);
    IUFillText(&FirmwareT[FW_RA], "RA", "", nullptr);
    IUFillText(&FirmwareT[FW_DEC], "DEC", "", nullptr);
    IUFillTextVector(&FirmwareTP, FirmwareT, 5, getDeviceName(), "Firmware Info", "", MOUNTINFO_TAB, IP_RO, 0,
                     IPS_IDLE);

    /* Tracking Mode */
    AddTrackMode("TRACK_SIDEREAL", "Sidereal", true);
    AddTrackMode("TRACK_SOLAR", "Solar");
    AddTrackMode("TRACK_LUNAR", "Lunar");
    AddTrackMode("TRACK_KING", "King");
    AddTrackMode("TRACK_CUSTOM", "Custom");

    // Slew Rates
    strncpy(SlewRateS[0].label, "1x", MAXINDILABEL);
    strncpy(SlewRateS[1].label, "2x", MAXINDILABEL);
    strncpy(SlewRateS[2].label, "8x", MAXINDILABEL);
    strncpy(SlewRateS[3].label, "16x", MAXINDILABEL);
    strncpy(SlewRateS[4].label, "64x", MAXINDILABEL);
    strncpy(SlewRateS[5].label, "128x", MAXINDILABEL);
    strncpy(SlewRateS[6].label, "256x", MAXINDILABEL);
    strncpy(SlewRateS[7].label, "512x", MAXINDILABEL);
    strncpy(SlewRateS[8].label, "MAX", MAXINDILABEL);
    IUResetSwitch(&SlewRateSP);
    // 64x is the default
    SlewRateS[4].s = ISS_ON;

    // Set TrackRate limits within +/- 0.0100 of Sidereal rate
    TrackRateN[AXIS_RA].min = TRACKRATE_SIDEREAL - 0.01;
    TrackRateN[AXIS_RA].max = TRACKRATE_SIDEREAL + 0.01;
    TrackRateN[AXIS_DE].min = -0.01;
    TrackRateN[AXIS_DE].max = 0.01;

    /* GPS Status */
    IUFillSwitch(&GPSStatusS[GPS_OFF], "Off", "", ISS_ON);
    IUFillSwitch(&GPSStatusS[GPS_ON], "On", "", ISS_OFF);
    IUFillSwitch(&GPSStatusS[GPS_DATA_OK], "Data OK", "", ISS_OFF);
    IUFillSwitchVector(&GPSStatusSP, GPSStatusS, 3, getDeviceName(), "GPS_STATUS", "GPS", MOUNTINFO_TAB, IP_RO,
                       ISR_1OFMANY, 0, IPS_IDLE);

    /* Time Source */
    IUFillSwitch(&TimeSourceS[TS_RS232], "RS232", "", ISS_ON);
    IUFillSwitch(&TimeSourceS[TS_CONTROLLER], "Controller", "", ISS_OFF);
    IUFillSwitch(&TimeSourceS[TS_GPS], "GPS", "", ISS_OFF);
    IUFillSwitchVector(&TimeSourceSP, TimeSourceS, 3, getDeviceName(), "TIME_SOURCE", "Time Source", MOUNTINFO_TAB,
                       IP_RO, ISR_1OFMANY, 0, IPS_IDLE);

    /* Hemisphere */
    IUFillSwitch(&HemisphereS[HEMI_SOUTH], "South", "", ISS_OFF);
    IUFillSwitch(&HemisphereS[HEMI_NORTH], "North", "", ISS_ON);
    IUFillSwitchVector(&HemisphereSP, HemisphereS, 2, getDeviceName(), "HEMISPHERE", "Hemisphere", MOUNTINFO_TAB, IP_RO,
                       ISR_1OFMANY, 0, IPS_IDLE);

    /* Home */
    IUFillSwitch(&HomeS[IEQ_SET_HOME], "IEQ_SET_HOME", "Set current as Home", ISS_OFF);
    IUFillSwitch(&HomeS[IEQ_GOTO_HOME], "IEQ_GOTO_HOME", "Go to Home", ISS_OFF);
    IUFillSwitch(&HomeS[IEQ_FIND_HOME], "IEQ_FIND_HOME", "Find Home", ISS_OFF);
    IUFillSwitchVector(&HomeSP, HomeS, 3, getDeviceName(), "HOME", "Home", MAIN_CONTROL_TAB, IP_RW, ISR_ATMOST1, 0,
                       IPS_IDLE);

    /* PEC */
    IUFillSwitch(&PECTrainingS[0], "PEC_Recording", "Record", ISS_OFF);
    IUFillSwitch(&PECTrainingS[1], "PEC_Status", "Status", ISS_OFF);
    IUFillSwitchVector(&PECTrainingSP, PECTrainingS, 2, getDeviceName(), "PEC_TRAINING", "PEC Training", PEC_TAB, IP_RW,
                       ISR_1OFMANY, 0,
                       IPS_IDLE);
    IUFillText(&PECInfoT[0], "PEC_INFO", "Status", "");
    IUFillTextVector(&PECInfoTP, PECInfoT, 1, getDeviceName(), "PEC_INFOS", "PEC Status", PEC_TAB,
                     IP_RO, 60, IPS_IDLE);
    IUFillText(&PECFileT[0], "PEC_FILE_PATH", "Path", getenv("HOME"));
    IUFillText(&PECFileT[1], "PEC_FILE_NAME", "Name", "pec.txt");
    IUFillText(&PECFileT[2], "PEC_FILE_TIME", "Time(YYYY-MM-DD hh:mm:ss)", "");
    IUFillTextVector(&PECFileTP, PECFileT, 3, getDeviceName(), "PEC_FILE", "PEC file", PEC_TAB,
                     IP_RW, 60, IPS_IDLE);
    IUFillNumber(&PECTimingN[0], "PEC_PERIOD", "Period (s)", "%g", 0, 1000, 1, 400);
    IUFillNumber(&PECTimingN[1], "PEC_DEPHASE", "Dephase (s)", "%g", 0, 1000, 1, 0);
    IUFillNumber(&PECTimingN[2], "PEC_MINSTEP", "Minimum pulse (ms)", "%g", 0, 1000, 1, 10);
    IUFillNumberVector(&PECTimingNP, PECTimingN, 3, getDeviceName(), "PEC_TIMING", "PEC Timing", PEC_TAB, IP_RW, 0,
                       IPS_IDLE);

    /* How fast do we guide compared to sidereal rate */
    IUFillNumber(&GuideRateN[RA_AXIS], "RA_GUIDE_RATE", "RA", "%.2f", 0.01, 0.9, 0.1, 0.5);
    IUFillNumber(&GuideRateN[DEC_AXIS], "DE_GUIDE_RATE", "DE", "%.2f", 0.1, 0.99, 0.1, 0.5);
    IUFillNumberVector(&GuideRateNP, GuideRateN, 2, getDeviceName(), "GUIDE_RATE", "Guiding Rate", MOTION_TAB, IP_RW, 0,
                       IPS_IDLE);

    TrackState = SCOPE_IDLE;

    initGuiderProperties(getDeviceName(), MOTION_TAB);

    setDriverInterface(getDriverInterface() | GUIDER_INTERFACE);

    SetParkDataType(PARK_AZ_ALT);

    addAuxControls();

    driver->setDeviceName(getDeviceName());

    // Only CEM40 and GEM45 have 115200 baud, rest are 9600
    if (strstr(getDeviceName(), "CEM40") || strstr(getDeviceName(), "GEM45"))
        serialConnection->setDefaultBaudRate(Connection::Serial::B_115200);

    double longitude = 0, latitude = 90;
    // Get value from config file if it exists.
    IUGetConfigNumber(getDeviceName(), "GEOGRAPHIC_COORD", "LONG", &longitude);
    currentRA  = get_local_sidereal_time(longitude);
    IUGetConfigNumber(getDeviceName(), "GEOGRAPHIC_COORD", "LAT", &latitude);
    currentDEC = latitude > 0 ? 90 : -90;

    return true;
}

bool IEQPro::updateProperties()
{

    if (isConnected())
    {
        getStartupData();

        INDI::Telescope::updateProperties();

        // Remove find home if we do not support it.
        if (!canFindHome)
            HomeSP.nsp = 2;

        defineProperty(&HomeSP);
        
        defineProperty(&PECTrainingSP);
        defineProperty(&PECInfoTP);
        defineProperty(&PECFileTP);
        defineProperty(&PECTimingNP);

        defineProperty(&GuideNSNP);
        defineProperty(&GuideWENP);

        if (canGuideRate)
            defineProperty(&GuideRateNP);

        defineProperty(&FirmwareTP);
        defineProperty(&GPSStatusSP);
        defineProperty(&TimeSourceSP);
        defineProperty(&HemisphereSP);
    }
    else
    {
        INDI::Telescope::updateProperties();

        HomeSP.nsp = 3;
        deleteProperty(HomeSP.name);

        deleteProperty(PECTrainingSP.name);
        deleteProperty(PECInfoTP.name);
        deleteProperty(PECFileTP.name);
        deleteProperty(PECTimingNP.name);

        deleteProperty(GuideNSNP.name);
        deleteProperty(GuideWENP.name);

        if (canGuideRate)
            deleteProperty(GuideRateNP.name);

        deleteProperty(FirmwareTP.name);
        deleteProperty(GPSStatusSP.name);
        deleteProperty(TimeSourceSP.name);
        deleteProperty(HemisphereSP.name);
    }

    return true;
}

void IEQPro::getStartupData()
{
    LOG_DEBUG("Getting firmware data...");

    firmwareInfo = driver->getFirmwareInfo();

    IUSaveText(&FirmwareT[0], firmwareInfo.Model.c_str());
    IUSaveText(&FirmwareT[1], firmwareInfo.MainBoardFirmware.c_str());
    IUSaveText(&FirmwareT[2], firmwareInfo.ControllerFirmware.c_str());
    IUSaveText(&FirmwareT[3], firmwareInfo.RAFirmware.c_str());
    IUSaveText(&FirmwareT[4], firmwareInfo.DEFirmware.c_str());

    FirmwareTP.s = IPS_OK;
    IDSetText(&FirmwareTP, nullptr);


    LOG_DEBUG("Getting guiding rate...");
    double raGuideRate = 0, deGuideRate = 0;
    if (driver->getGuideRate(&raGuideRate, &deGuideRate))
    {
        GuideRateN[RA_AXIS].value = raGuideRate;
        GuideRateN[DEC_AXIS].value = deGuideRate;
        IDSetNumber(&GuideRateNP, nullptr);
    }

    double utc_offset;
    int yy, dd, mm, hh, minute, ss;
    if (driver->getUTCDateTime(&utc_offset, &yy, &mm, &dd, &hh, &minute, &ss))
    {
        char isoDateTime[32] = {0};
        char utcOffset[8] = {0};

        snprintf(isoDateTime, 32, "%04d-%02d-%02dT%02d:%02d:%02d", yy, mm, dd, hh, minute, ss);
        snprintf(utcOffset, 8, "%4.2f", utc_offset);

        IUSaveText(IUFindText(&TimeTP, "UTC"), isoDateTime);
        IUSaveText(IUFindText(&TimeTP, "OFFSET"), utcOffset);

        LOGF_INFO("Mount UTC offset is %s. UTC time is %s", utcOffset, isoDateTime);

        TimeTP.s = IPS_OK;
        IDSetText(&TimeTP, nullptr);
    }

    // Get Longitude and Latitude from mount
    double longitude = 0, latitude = 0;
    if (driver->getStatus(&scopeInfo))
    {
        longitude = scopeInfo.longitude;
        latitude = scopeInfo.latitude;

        // Convert to INDI standard longitude (0 to 360 Eastward)
        if (longitude < 0)
            longitude += 360;

        LOGF_INFO("Mount Longitude %g Latitude %g", longitude, latitude);

        LocationN[LOCATION_LATITUDE].value  = latitude;
        LocationN[LOCATION_LONGITUDE].value = longitude;
        LocationNP.s                        = IPS_OK;

        IDSetNumber(&LocationNP, nullptr);

        saveConfig(true, "GEOGRAPHIC_COORD");
    }
    else if (IUGetConfigNumber(getDeviceName(), "GEOGRAPHIC_COORD", "LONG", &longitude) == 0 &&
             IUGetConfigNumber(getDeviceName(), "GEOGRAPHIC_COORD", "LAT", &latitude) == 0)
    {
        LocationN[LOCATION_LATITUDE].value  = latitude;
        LocationN[LOCATION_LONGITUDE].value = longitude;
        LocationNP.s                        = IPS_OK;

        IDSetNumber(&LocationNP, nullptr);
    }

    if (InitPark())
    {
        // If loading parking data is successful, we just set the default parking values.
        SetAxis1ParkDefault(LocationN[LOCATION_LATITUDE].value >= 0 ? 0 : 180);
        SetAxis2ParkDefault(LocationN[LOCATION_LATITUDE].value);
    }
    else
    {
        // Otherwise, we set all parking data to default in case no parking data is found.
        SetAxis1Park(LocationN[LOCATION_LATITUDE].value >= 0 ? 0 : 180);
        SetAxis2Park(LocationN[LOCATION_LATITUDE].value);
        SetAxis1ParkDefault(LocationN[LOCATION_LATITUDE].value >= 0 ? 0 : 180);
        SetAxis2ParkDefault(LocationN[LOCATION_LATITUDE].value);
    }

    // can we read pier side?
    IEQ_PIER_SIDE pierSide = IEQ_PIER_UNKNOWN;
    if (driver->getPierSide(&pierSide) && pierSide != IEQ_PIER_UNKNOWN)
    {
        // add the pier side capability
        auto cap = GetTelescopeCapability();
        cap |= TELESCOPE_HAS_PIER_SIDE;
        SetTelescopeCapability(cap, 9);
    }

   /* v3.0 Read PEC State at startup */
    iEQ::Base::Info newInfo;
    if (driver->getStatus(&newInfo))
    {
        switch (newInfo.systemStatus)
        {
            case ST_STOPPED:
            case ST_PARKED:
            case ST_HOME:
            case ST_SLEWING:
            case ST_MERIDIAN_FLIPPING:
            case ST_GUIDING:

            case ST_TRACKING_PEC_OFF:
                setPECState(PEC_OFF);
                break;

            case ST_TRACKING_PEC_ON:
                setPECState(PEC_ON);
                break;
        }
        scopeInfo = newInfo;
    }
    // End Mod */

    //    if (isSimulation())
    //    {
    //        if (isParked())
    //            set_sim_system_status(ST_PARKED);
    //        else
    //            set_sim_system_status(ST_STOPPED);
    //    }
}

bool IOptronV3::ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        // PEC file
        if (!strcmp(name, PECFileTP.name))
        {
            snprintf(PECFullPath,256,"%s/%s",texts[0],texts[1]);
            

            int day, month, year, hour, min, sec;
            sscanf(texts[2],"%4d-%2d-%2d %2d:%2d:%2d",&year,&month,&day,&hour,&min,&sec);
            PECStart_tm.tm_year = year - 1900;
            PECStart_tm.tm_mon = month - 1;
            PECStart_tm.tm_mday = day;
            PECStart_tm.tm_hour = hour;
            PECStart_tm.tm_min = min;
            PECStart_tm.tm_sec = sec;
            PECStart_tm.tm_isdst = -1;
            PECStartTime = mktime(&PECStart_tm);

            char temp_string[64];
            snprintf(temp_string,64,"PEC file path: %s",PECFullPath);
            LOG_INFO(temp_string);
            
            snprintf(temp_string,64,"Acquisition time: %s",asctime(&PECStart_tm));
            LOG_INFO(temp_string);


            if ((int)(PECTimingN[1].value) !=0)
            {
                PECStart_tm.tm_sec += PECTimingN[1].value;
                PECStartTime = mktime(&PECStart_tm);
                snprintf(temp_string,64,"Acquisition time dephased: %s",asctime(&PECStart_tm));
                LOG_INFO(temp_string);           
            }

            time_t actual_time;
            time(&actual_time);
            snprintf(temp_string,64,"Actual time: %s",ctime(&actual_time));
            LOG_INFO(temp_string);     

            int nextPeriodDelta;
            nextPeriodDelta=(int)((floor(difftime(actual_time, PECStartTime)/PECTimingN[0].value)+1)*PECTimingN[0].value-
                                        difftime(actual_time, PECStartTime));
            snprintf(temp_string,50,"Time to next period start: %ds",nextPeriodDelta);
            LOG_INFO(temp_string);

            snprintf(texts[2],MAXINDILABEL,"%s",asctime(&PECStart_tm));
            
            IUUpdateText(&PECFileTP, texts, names, n);
            PECFileTP.s = IPS_OK;
            IDSetText(&PECFileTP, nullptr);

            return true;
        }
    }

    return INDI::Telescope::ISNewText(dev, name, texts, names, n);
}

bool IEQPro::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        // Guiding Rate
        if (!strcmp(name, GuideRateNP.name))
        {
            IUUpdateNumber(&GuideRateNP, values, names, n);

            if (driver->setGuideRate(GuideRateN[RA_AXIS].value, GuideRateN[DEC_AXIS].value))
                GuideRateNP.s = IPS_OK;
            else
                GuideRateNP.s = IPS_ALERT;

            IDSetNumber(&GuideRateNP, nullptr);

            return true;
        }

        // PEC
        if (!strcmp(name, PECTimingNP.name))
        {
            IUUpdateNumber(&PECTimingNP, values, names, n);
            PECTimingNP.s = IPS_OK;
            IDSetNumber(&PECTimingNP, nullptr);
            return true;
        }

        if (!strcmp(name, GuideNSNP.name) || !strcmp(name, GuideWENP.name))
        {
            processGuiderProperties(name, values, names, n);
            return true;
        }
    }

    return INDI::Telescope::ISNewNumber(dev, name, values, names, n);
}

bool IEQPro::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    if (!strcmp(getDeviceName(), dev))
    {
        if (!strcmp(name, HomeSP.name))
        {
            IUUpdateSwitch(&HomeSP, states, names, n);

            HomeOperation operation = static_cast<HomeOperation>(IUFindOnSwitchIndex(&HomeSP));

            IUResetSwitch(&HomeSP);

            switch (operation)
            {
                case IEQ_SET_HOME:
                    if (driver->setCurrentHome() == false)
                    {
                        HomeSP.s = IPS_ALERT;
                        IDSetSwitch(&HomeSP, nullptr);
                        return false;
                    }

                    HomeSP.s = IPS_OK;
                    IDSetSwitch(&HomeSP, nullptr);
                    LOG_INFO("Home position set to current coordinates.");
                    return true;

                case IEQ_GOTO_HOME:
                    if (TrackState == SCOPE_PARKED)
                    {
                        LOG_ERROR("Please unpark the mount before issuing any motion commands.");
                        return false;
                    }

                    if (driver->gotoHome() == false)
                    {
                        HomeSP.s = IPS_ALERT;
                        IDSetSwitch(&HomeSP, nullptr);
                        return false;
                    }

                    HomeSP.s = IPS_OK;
                    IDSetSwitch(&HomeSP, nullptr);
                    LOG_INFO("Slewing to home position...");
                    return true;

                case IEQ_FIND_HOME:
                    if (driver->findHome() == false)
                    {
                        HomeSP.s = IPS_ALERT;
                        IDSetSwitch(&HomeSP, nullptr);
                        return false;
                    }

                    HomeSP.s = IPS_OK;
                    IDSetSwitch(&HomeSP, nullptr);
                    LOG_INFO("Searching for home position...");
                    return true;
            }

            return true;
        }
    

        /*******************************************************
         * PEC Operations
        *******************************************************/
        
        if (!strcmp(name, PECStateSP.name))
        {
            IUUpdateSwitch(&PECStateSP, states, names, n);

            if(IUFindOnSwitchIndex(&PECStateSP) == 0)
            {
                // PEC OFF
                if(isTraining)
                {
                    // Training check
                    sprintf(PECText, "Mount PEC busy recording index %d", PECIndex);
                    LOG_WARN(PECText);
                }
                else
                {
                    driver->setPECEnabled(false);
                    PECStateSP.s = IPS_OK;
                    LOG_INFO("Disabling PEC Chip");
                }
            }

            if(IUFindOnSwitchIndex(&PECStateSP) == 1)
            {   
                driver->setPECEnabled(true);
                PECStateSP.s = IPS_BUSY;
                LOG_INFO("Enabling PEC Chip");
            }
            
            IDSetSwitch(&PECStateSP, nullptr);
            return true;
        }

        /*******************************************************
         * PEC Training
        *******************************************************/
        if (!strcmp(name, PECTrainingSP.name))
        {
            IUUpdateSwitch(&PECTrainingSP, states, names, n);
            if (isTraining || isWaitingTraining)
            {
                // Check if already training
                if(IUFindOnSwitchIndex(&PECTrainingSP) == 1)
                {
                    // Train Check Status
                    sprintf(PECText, "Mount PEC busy recording index %d", PECIndex);
                    LOG_WARN(PECText);
                }

                if(IUFindOnSwitchIndex(&PECTrainingSP) == 0)
                {
                    // Train Cancel
                    driver->setPETEnabled(false);
                    delete[] PECvalues;
                    isTraining = false;
                    isWaitingTraining = false;
                    PECTrainingSP.s = IPS_ALERT;
                    LOG_WARN("PEC Training cancelled by user, chip disabled");
                }
            }
            else
            {
                if(IUFindOnSwitchIndex(&PECTrainingSP) == 0)
                {
                    if(TrackState == SCOPE_TRACKING)
                    {
                        // Train if tracking /guiding
                        PECfile.open(PECFullPath);
                        std::string str; 

                        std::getline(PECfile, str);                   
                        std::getline(PECfile, str);                      
                        std::getline(PECfile, str);                      
                        std::getline(PECfile, str);                      
                        std::getline(PECfile, str);
                        
                        PECvalues = new float[(int)PECTimingN[0].value];

                        for (int i=0; i<PECTimingN[0].value; i++)
                        {
                            std::getline(PECfile, str);
                            PECvalues[i] = std::stof(str.substr(str.find_last_of(" ")));
                            LOG_INFO(std::to_string(PECvalues[i]).c_str());
                        }
                        
                        PECfile.close();

                        isWaitingTraining = true;
                        
                        PECTrainingSP.s = IPS_BUSY;
                        int nextPeriodDelta;
                        char temp_string[64];
                        time_t actual_time;
                        time(&actual_time);
                        nextPeriodDelta=(int)((floor(difftime(actual_time, PECStartTime)/PECTimingN[0].value)+1)*PECTimingN[0].value-
                                            difftime(actual_time, PECStartTime));
                        snprintf(temp_string,50,"PEC recording started, time to next period: %ds",nextPeriodDelta);
                        LOG_INFO(temp_string);                      
                    }
                    else
                    {
                        LOG_WARN("PEC Training only possible while guiding");
                        PECTrainingSP.s = IPS_IDLE;
                    }
                }
                if(IUFindOnSwitchIndex(&PECTrainingSP) == 1)
                {
                    // Train Status
                    LOG_WARN("PEC Data status not supported");
                }
            }
            IDSetSwitch(&PECTrainingSP, nullptr);
            return true;
        }
    }

    return INDI::Telescope::ISNewSwitch(dev, name, states, names, n);
}

bool IEQPro::ReadScopeStatus()
{
    iEQ::Base::Info newInfo;

    //    if (isSimulation())
    //        mountSim();

    bool rc = driver->getStatus(&newInfo);

    if (rc)
    {
        IUResetSwitch(&GPSStatusSP);
        GPSStatusS[newInfo.gpsStatus].s = ISS_ON;
        IDSetSwitch(&GPSStatusSP, nullptr);

        IUResetSwitch(&TimeSourceSP);
        TimeSourceS[newInfo.timeSource].s = ISS_ON;
        IDSetSwitch(&TimeSourceSP, nullptr);

        IUResetSwitch(&HemisphereSP);
        HemisphereS[newInfo.hemisphere].s = ISS_ON;
        IDSetSwitch(&HemisphereSP, nullptr);

        /*
        TelescopeTrackMode trackMode = TRACK_SIDEREAL;

        switch (newInfo.trackRate)
        {
            case TR_SIDEREAL:
                trackMode = TRACK_SIDEREAL;
                break;
            case TR_SOLAR:
                trackMode = TRACK_SOLAR;
                break;
            case TR_LUNAR:
                trackMode = TRACK_LUNAR;
                break;
            case TR_KING:
                trackMode = TRACK_SIDEREAL;
                break;
            case TR_CUSTOM:
                trackMode = TRACK_CUSTOM;
                break;
        }*/

        switch (newInfo.systemStatus)
        {
            case ST_STOPPED:
                TrackModeSP.s = IPS_IDLE;
                // If we cannot park natively and we already parked
                // scope, we do not want its state to change to IDLE
                // For scopes that can park natively, ST_PARKED would be
                // set already.
                if (canParkNatively || TrackState != SCOPE_PARKED)
                    TrackState    = SCOPE_IDLE;
                break;
            case ST_PARKED:
                TrackModeSP.s = IPS_IDLE;
                TrackState    = SCOPE_PARKED;
                if (!isParked())
                    SetParked(true);
                break;
            case ST_HOME:
                TrackModeSP.s = IPS_IDLE;
                TrackState    = SCOPE_IDLE;
                break;
            case ST_SLEWING:
            case ST_MERIDIAN_FLIPPING:
                slewDirty = true;
                if (TrackState != SCOPE_SLEWING && TrackState != SCOPE_PARKING)
                    TrackState = SCOPE_SLEWING;
                break;
            /* v3.0 PEC update status */
            case ST_TRACKING_PEC_OFF:
            case ST_TRACKING_PEC_ON:
            case ST_GUIDING:
                if (newInfo.systemStatus == ST_TRACKING_PEC_OFF || newInfo.systemStatus == ST_TRACKING_PEC_ON)
                    setPECState(newInfo.systemStatus == ST_TRACKING_PEC_ON ? PEC_ON : PEC_OFF);
                if (TrackState == SCOPE_PARKING && canParkNatively == false)
                {
                    if (slewDirty)
                    {
                        LOG_INFO("Manual parking complete. Shut the mount down.");
                        TrackModeSP.s = IPS_IDLE;
                        TrackState    = SCOPE_PARKED;
                        SetTrackEnabled(false);
                        SetParked(true);
                        slewDirty = false;
                    }
                }
                else
                {
                    TrackModeSP.s = IPS_BUSY;
                    TrackState    = SCOPE_TRACKING;
                    if (scopeInfo.systemStatus == ST_SLEWING)
                        LOG_INFO("Slew complete, tracking...");
                    else if (scopeInfo.systemStatus == ST_MERIDIAN_FLIPPING)
                        LOG_INFO("Meridian flip complete, tracking...");
                }
                break;
        }

        IUResetSwitch(&TrackModeSP);
        TrackModeS[newInfo.trackRate].s = ISS_ON;
        IDSetSwitch(&TrackModeSP, nullptr);

        scopeInfo = newInfo;
    }

    /* v3.0 Monitor PEC Training */
    if (isTraining)
    {
        if (TrackState == SCOPE_TRACKING)
        {
            if(elapsed_training > (int)(PECTimingN[0].value) + 10))
            {
                sprintf(PECText, "PEC with %d second worm cycle recorded", (int)(PECTimingN[0].value));
                LOG_INFO(PECText);
                delete[] PECvalues;
                PECTrainingSP.s = IPS_OK;
                isTraining = false;
            }
            else
            {
                char temp_str[128];
                time_t actual_time;
  
                time(&actual_time);
                int nextPeriodDelta;
                nextPeriodDelta=(int)((floor(difftime(actual_time, PECStartTime)/PECTimingN[0].value)+1)*PECTimingN[0].value-
                                        difftime(actual_time, PECStartTime));

                PECIndex = PECTimingN[0].value-nextPeriodDelta + PECTimingN[1].value;

                if (PECIndex>=PECTimingN[0].value)
                    PECIndex = PECIndex - PECTimingN[0].value;

                float deltaPE;
                int guidePulse;

                deltaPE= PECvalues[PECIndex]-appliedCorrection;
                guidePulse = (int)floor((deltaPE/(TRACKRATE_SIDEREAL*GuideRateN[0].value))*1000 + 0.5);   //round to ms
                if (guidePulse > (PECTimingN[2].value) || guidePulse<-(PECTimingN[2].value))
                {
                    //apply guiding here
                    if (guidePulse > 0)
                        GuideWest((int)(guidePulse));
                    else
                        GuideEast((int)(-guidePulse));

                    appliedCorrection += (guidePulse/1000.0) * (TRACKRATE_SIDEREAL*GuideRateN[0].value);
                }
                else
                {
                    guidePulse = 0;
                }

                elapsed_training = elapsed_training + getCurrentPollingPeriod()/1000;

                sprintf(temp_str, "Index: %d, delta: %.4f, pulse: %d, total corr: %.4f, error: %.4f",
                             PECIndex, deltaPE, guidePulse, appliedCorrection, PECvalues[PECIndex]-appliedCorrection);
                LOG_INFO(temp_str);

                sprintf(PECText, "Recording index %d", PECIndex);
                IUSaveText(&PECInfoT[0], PECText);
            }
        }
        else
        {
            driver->setPETEnabled(false);
            PECTrainingSP.s = IPS_ALERT;
            sprintf(PECText, "Tracking error, recording cancelled at index %d", PECIndex);
            delete[] PECvalues;
            LOG_ERROR(PECText);
            IUSaveText(&PECInfoT[0], "Cancelled");
        }
        IDSetText(&PECInfoTP, nullptr);
        IDSetSwitch(&PECTrainingSP, nullptr);
    }
    
    if (isWaitingTraining)
    {
        if (TrackState == SCOPE_TRACKING)
        {
            time_t actual_time;
  
            time(&actual_time);
            

            int nextPeriodDelta;
            nextPeriodDelta=(int)((floor(difftime(actual_time, PECStartTime)/PECTimingN[0].value)+1)*PECTimingN[0].value-
                                        difftime(actual_time, PECStartTime));

            if (nextPeriodDelta == PECTimingN[0].value)
            {
                driver->setPETEnabled(true);
                isWaitingTraining = false;
                isTraining = true;
                elapsed_training = 0;
                LOG_INFO("Recording started.");
                PECIndex = PECTimingN[1].value;;
                appliedCorrection = 0;
                sprintf(PECText, "Recording index %d", PECIndex);
                IUSaveText(&PECInfoT[0], PECText);
            }
            else
            {
                sprintf(PECText, "Time to start recording: %d s", nextPeriodDelta);
                IUSaveText(&PECInfoT[0], PECText);
            }
            
        }
        else
        {
            driver->setPETEnabled(false);
            PECTrainingSP.s = IPS_ALERT;
            sprintf(PECText, "Tracking error, recording cancelled at index %d", PECIndex);
            delete[] PECvalues;
            isWaitingTraining = false;
            isTraining = false;
            LOG_ERROR(PECText);
            IUSaveText(&PECInfoT[0], "Cancelled");
        }
        IDSetText(&PECInfoTP, nullptr);
        IDSetSwitch(&PECTrainingSP, nullptr);
    }
    // End Mod */

    if (HasPierSide())
    {
        IEQ_PIER_SIDE pierSide;
        if (driver->getPierSide(&pierSide))
        {
            TelescopePierSide tps = PIER_UNKNOWN;
            switch (pierSide)
            {
                case IEQ_PIER_UNKNOWN:
                case IEQ_PIER_UNCERTAIN:
                    tps = PIER_UNKNOWN;
                    break;
                case IEQ_PIER_EAST:
                    tps = PIER_EAST;
                    break;
                case IEQ_PIER_WEST:
                    tps = PIER_WEST;
                    break;
            }
            setPierSide(tps);
        }
    }

    rc = driver->getCoords(&currentRA, &currentDEC);

    if (rc)
        NewRaDec(currentRA, currentDEC);

    return rc;
}

bool IEQPro::Goto(double r, double d)
{
    targetRA  = r;
    targetDEC = d;
    char RAStr[64] = {0}, DecStr[64] = {0};

    fs_sexa(RAStr, targetRA, 2, 3600);
    fs_sexa(DecStr, targetDEC, 2, 3600);

    if (driver->setRA(r) == false || driver->setDE(d) == false)
    {
        LOG_ERROR("Error setting RA/DEC.");
        return false;
    }

    if (driver->slew() == false)
    {
        LOG_ERROR("Failed to slew.");
        return false;
    }

    iEQ::Base::Info newInfo;

    // Wait until the mount system status changes to SLEWING
    // up to 500ms
    for (int i = 0; i < 5; i++)
    {
        bool rc = driver->getStatus(&newInfo);
        if (rc && newInfo.systemStatus == ST_SLEWING)
            break;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (newInfo.systemStatus == ST_SLEWING)
    {
        TrackState = SCOPE_SLEWING;
        LOGF_INFO("Slewing to RA: %s - DEC: %s", RAStr, DecStr);
        return true;
    }
    else
    {
        LOG_ERROR("Mount status failed to update to slewing.");
        return false;
    }
}

bool IEQPro::Sync(double ra, double dec)
{
    if (driver->setRA(ra) == false || driver->setDE(dec) == false)
    {
        LOG_ERROR("Error setting RA/DEC.");
        return false;
    }

    if (driver->sync() == false)
    {
        LOG_ERROR("Failed to sync.");
    }

    EqNP.s     = IPS_OK;

    currentRA  = ra;
    currentDEC = dec;

    NewRaDec(currentRA, currentDEC);

    return true;
}

bool IEQPro::Abort()
{
    return driver->abort();
}

bool IEQPro::Park()
{
#if 0
    targetRA  = GetAxis1Park();
    targetDEC = GetAxis2Park();

    if (set_ieqpro_ra(PortFD, targetRA) == false || set_ieqpro_dec(PortFD, targetDEC) == false)
    {
        LOG_ERROR("Error setting RA/DEC.");
        return false;
    }

    if (slew_ieqpro(PortFD) == false)
    {
        LOG_ERROR("Failed to slew tp parking position.");
        return false;
    }

    char RAStr[64] = {0}, DecStr[64] = {0};
    fs_sexa(RAStr, targetRA, 2, 3600);
    fs_sexa(DecStr, targetDEC, 2, 3600);

    TrackState = SCOPE_PARKING;
    LOGF_INFO("Telescope parking in progress to RA: %s DEC: %s", RAStr, DecStr);

    return true;
#endif

    double parkAz  = GetAxis1Park();
    double parkAlt = GetAxis2Park();

    char AzStr[16], AltStr[16];
    fs_sexa(AzStr, parkAz, 2, 3600);
    fs_sexa(AltStr, parkAlt, 2, 3600);
    LOGF_DEBUG("Parking to Az (%s) Alt (%s)...", AzStr, AltStr);

    // Check if mount supports native Alt/Az parking
    if (canParkNatively)
    {
        if (driver->setParkAz(parkAz) && driver->setParkAlt(parkAlt))
        {
            if (driver->park())
            {
                TrackState = SCOPE_PARKING;
                LOG_INFO("Parking is in progress...");
                return true;
            }
            else
                return false;
        }
        else
            return false;
    }

    // Otherwise fallback to Alt/Az --> RA/DE parking
    ln_lnlat_posn observer;
    observer.lat = LocationN[LOCATION_LATITUDE].value;
    observer.lng = LocationN[LOCATION_LONGITUDE].value;
    if (observer.lng > 180)
        observer.lng -= 360;

    ln_hrz_posn horizontalPos;
    // Libnova south = 0, west = 90, north = 180, east = 270

    horizontalPos.az = parkAz;
    horizontalPos.alt = parkAlt;
    ln_equ_posn equatorialPos;

    get_equ_from_hrz(&horizontalPos, &observer, ln_get_julian_from_sys(), &equatorialPos);

    if (Goto(equatorialPos.ra / 15.0, equatorialPos.dec))
    {
        TrackState = SCOPE_PARKING;
        LOG_INFO("Parking is in progress...");

        slewDirty = false;
        return true;
    }
    else
        return false;
}

bool IEQPro::UnPark()
{
    // If we can park natively, let's try to unpark the mount first.
    // If that fails, we return. Otherwise, we proceed even when
    // we are manually unparking.
    if (canParkNatively && driver->unpark() == false)
        return false;

    SetParked(false);
    TrackState = SCOPE_IDLE;
    return true;
}

bool IEQPro::Handshake()
{
    //    if (isSimulation())
    //    {
    //        set_sim_gps_status(GPS_DATA_OK);
    //        set_sim_system_status(ST_STOPPED);
    //        set_sim_track_rate(TR_SIDEREAL);
    //        set_sim_slew_rate(SR_3);
    //        set_sim_time_source(TS_GPS);
    //        set_sim_hemisphere(HEMI_NORTH);
    //    }

    if (driver->initCommunication(PortFD) == false)
        return false;

    canParkNatively = driver->isCommandSupported("MP1", true);
    canFindHome = driver->isCommandSupported("MSH", true);
    canGuideRate = driver->isCommandSupported("RG", true);

    return true;
}

bool IEQPro::updateTime(ln_date *utc, double utc_offset)
{
    struct ln_zonedate ltm;

    ln_date_to_zonedate(utc, &ltm, utc_offset * 3600.0);

    // Set Local Time
    if (driver->setLocalTime(ltm.hours, ltm.minutes, ltm.seconds) == false)
    {
        LOG_ERROR("Error setting local time.");
        return false;
    }

    // Send it as YY (i.e. 2015 --> 15)
    ltm.years -= 2000;

    // Set Local date
    if (driver->setLocalDate(ltm.years, ltm.months, ltm.days) == false)
    {
        LOG_ERROR("Error setting local date.");
        return false;
    }

    // UTC Offset
    if (driver->setUTCOffset(utc_offset) == false)
    {
        LOG_ERROR("Error setting UTC Offset.");
        return false;
    }

    LOG_INFO("Time and date updated.");

    return true;
}

bool IEQPro::updateLocation(double latitude, double longitude, double elevation)
{
    INDI_UNUSED(elevation);

    if (longitude > 180)
        longitude -= 360;

    if (driver->setLongitude(longitude) == false)
    {
        LOG_ERROR("Failed to set longitude.");
        return false;
    }

    if (driver->setLatitude(latitude) == false)
    {
        LOG_ERROR("Failed to set latitude.");
        return false;
    }

    char l[32] = {0}, L[32] = {0};
    fs_sexa(l, latitude, 3, 3600);
    fs_sexa(L, longitude, 4, 3600);

    LOGF_INFO("Site location updated to Lat %.32s - Long %.32s", l, L);

    return true;
}

void IEQPro::debugTriggered(bool enable)
{
    driver->setDebugEnabled(enable);
}

void IEQPro::simulationTriggered(bool enable)
{
    INDI_UNUSED(enable);
    //driver->setSi(enable);
}

bool IEQPro::MoveNS(INDI_DIR_NS dir, TelescopeMotionCommand command)
{
    if (TrackState == SCOPE_PARKED)
    {
        LOG_ERROR("Please unpark the mount before issuing any motion commands.");
        return false;
    }

    switch (command)
    {
        case MOTION_START:
            if (driver->startMotion((dir == DIRECTION_NORTH ? IEQ_N : IEQ_S)) == false)
            {
                LOG_ERROR("Error setting N/S motion direction.");
                return false;
            }
            else
                LOGF_INFO("Moving toward %s.", (dir == DIRECTION_NORTH) ? "North" : "South");
            break;

        case MOTION_STOP:
            if (driver->stopMotion((dir == DIRECTION_NORTH ? IEQ_N : IEQ_S)) == false)
            {
                LOG_ERROR("Error stopping N/S motion.");
                return false;
            }
            else
                LOGF_INFO("%s motion stopped.", (dir == DIRECTION_NORTH) ? "North" : "South");
            break;
    }

    return true;
}

bool IEQPro::MoveWE(INDI_DIR_WE dir, TelescopeMotionCommand command)
{
    if (TrackState == SCOPE_PARKED)
    {
        LOG_ERROR("Please unpark the mount before issuing any motion commands.");
        return false;
    }

    switch (command)
    {
        case MOTION_START:
            if (driver->startMotion((dir == DIRECTION_WEST ? IEQ_W : IEQ_E)) == false)
            {
                LOG_ERROR("Error setting N/S motion direction.");
                return false;
            }
            else
                LOGF_INFO("Moving toward %s.", (dir == DIRECTION_WEST) ? "West" : "East");
            break;

        case MOTION_STOP:
            if (driver->stopMotion((dir == DIRECTION_WEST ? IEQ_W : IEQ_E)) == false)
            {
                LOG_ERROR("Error stopping W/E motion.");
                return false;
            }
            else
                LOGF_INFO("%s motion stopped.", (dir == DIRECTION_WEST) ? "West" : "East");
            break;
    }

    return true;
}

IPState IEQPro::GuideNorth(uint32_t ms)
{
    bool rc = driver->startGuide(IEQ_N, ms);
    return (rc ? IPS_OK : IPS_ALERT);
}

IPState IEQPro::GuideSouth(uint32_t ms)
{
    bool rc = driver->startGuide(IEQ_S, ms);
    return (rc ? IPS_OK : IPS_ALERT);
}

IPState IEQPro::GuideEast(uint32_t ms)
{
    bool rc = driver->startGuide(IEQ_E, ms);
    return (rc ? IPS_OK : IPS_ALERT);
}

IPState IEQPro::GuideWest(uint32_t ms)
{
    bool rc = driver->startGuide(IEQ_W, ms);
    return (rc ? IPS_OK : IPS_ALERT);
}

bool IEQPro::SetSlewRate(int index)
{
    SlewRate rate = static_cast<SlewRate>(index);
    return driver->setSlewRate(rate);
}

bool IEQPro::saveConfigItems(FILE *fp)
{
    INDI::Telescope::saveConfigItems(fp);

    return true;
}

//void IEQPro::mountSim()
//{
//    static struct timeval ltv;
//    struct timeval tv;
//    double dt, da, dx;
//    int nlocked;

//    /* update elapsed time since last poll, don't presume exactly POLLMS */
//    gettimeofday(&tv, nullptr);

//    if (ltv.tv_sec == 0 && ltv.tv_usec == 0)
//        ltv = tv;

//    dt  = tv.tv_sec - ltv.tv_sec + (tv.tv_usec - ltv.tv_usec) / 1e6;
//    ltv = tv;
//    da  = SLEWRATE * dt;

//    /* Process per current state. We check the state of EQUATORIAL_COORDS and act acoordingly */
//    switch (TrackState)
//    {
//        case SCOPE_IDLE:
//            currentRA += (TrackRateN[AXIS_RA].value / 3600.0 * dt) / 15.0;
//            currentRA = range24(currentRA);
//            break;

//        case SCOPE_TRACKING:
//            if (TrackModeS[1].s == ISS_ON)
//            {
//                currentRA  += ( ((TRACKRATE_SIDEREAL / 3600.0) - (TrackRateN[AXIS_RA].value / 3600.0)) * dt) / 15.0;
//                currentDEC += ( (TrackRateN[AXIS_DE].value / 3600.0) * dt);
//            }
//            break;

//        case SCOPE_SLEWING:
//        case SCOPE_PARKING:
//            /* slewing - nail it when both within one pulse @ SLEWRATE */
//            nlocked = 0;

//            dx = targetRA - currentRA;

//            // Take shortest path
//            if (fabs(dx) > 12)
//                dx *= -1;

//            if (fabs(dx) <= da)
//            {
//                currentRA = targetRA;
//                nlocked++;
//            }
//            else if (dx > 0)
//                currentRA += da / 15.;
//            else
//                currentRA -= da / 15.;

//            if (currentRA < 0)
//                currentRA += 24;
//            else if (currentRA > 24)
//                currentRA -= 24;

//            dx = targetDEC - currentDEC;
//            if (fabs(dx) <= da)
//            {
//                currentDEC = targetDEC;
//                nlocked++;
//            }
//            else if (dx > 0)
//                currentDEC += da;
//            else
//                currentDEC -= da;

//            if (nlocked == 2)
//            {
//                if (TrackState == SCOPE_SLEWING)
//                    set_sim_system_status(ST_TRACKING_PEC_OFF);
//                else
//                    set_sim_system_status(ST_PARKED);
//            }

//            break;

//        default:
//            break;
//    }

//    set_sim_ra(currentRA);
//    set_sim_dec(currentDEC);
//}

bool IEQPro::SetCurrentPark()
{
    ln_hrz_posn horizontalPos;
    // Libnova south = 0, west = 90, north = 180, east = 270

    ln_lnlat_posn observer;
    observer.lat = LocationN[LOCATION_LATITUDE].value;
    observer.lng = LocationN[LOCATION_LONGITUDE].value;
    if (observer.lng > 180)
        observer.lng -= 360;

    ln_equ_posn equatorialPos;
    equatorialPos.ra  = currentRA * 15;
    equatorialPos.dec = currentDEC;
    get_hrz_from_equ(&equatorialPos, &observer, ln_get_julian_from_sys(), &horizontalPos);
    double parkAZ = horizontalPos.az;
    double parkAlt = horizontalPos.alt;

    char AzStr[16], AltStr[16];
    fs_sexa(AzStr, parkAZ, 2, 3600);
    fs_sexa(AltStr, parkAlt, 2, 3600);

    LOGF_DEBUG("Setting current parking position to coordinates Az (%s) Alt (%s)...", AzStr,
               AltStr);

    SetAxis1Park(parkAZ);
    SetAxis2Park(parkAlt);

    return true;
}

bool IEQPro::SetDefaultPark()
{
    // By defualt azimuth 0
    SetAxis1Park(0);

    // Altitude = latitude of observer
    SetAxis2Park(LocationN[LOCATION_LATITUDE].value);

    return true;
}

bool IEQPro::SetTrackMode(uint8_t mode)
{
    TrackRate rate = static_cast<TrackRate>(mode);

    if (driver->setTrackMode(rate))
        return true;

    return false;
}

bool IEQPro::SetTrackRate(double raRate, double deRate)
{
    static bool deRateWarning = true;

    // Convert to arcsecs/s to +/- 0.0100 accepted by
    double ieqRARate = raRate - TRACKRATE_SIDEREAL;
    if (deRate != 0 && deRateWarning)
    {
        // Only send warning once per session
        deRateWarning = false;
        LOG_WARN("Custom Declination tracking rate is not implemented yet.");
    }

    if (driver->setCustomRATrackRate(ieqRARate))
        return true;

    return false;
}

bool IEQPro::SetTrackEnabled(bool enabled)
{
    if (enabled)
    {
        // If we are engaging tracking, let us first set tracking mode, and if we have custom mode, then tracking rate.
        // NOTE: Is this the correct order? or should tracking be switched on first before making these changes? Need to test.
        SetTrackMode(IUFindOnSwitchIndex(&TrackModeSP));
        if (TrackModeS[TR_CUSTOM].s == ISS_ON)
            SetTrackRate(TrackRateN[AXIS_RA].value, TrackRateN[AXIS_DE].value);
    }

    return driver->setTrackEnabled(enabled);
}