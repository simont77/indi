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

#include "ieqdriverbase.h"

#include "indicom.h"
#include "indilogger.h"

#include <libnova/julian_day.h>

#include <cmath>
#include <map>
#include <cstring>
#include <algorithm>
#include <termios.h>
#include <unistd.h>

namespace iEQ
{

Base::Base()
{
}

bool Base::initCommunication(int fd)
{
    m_PortFD = fd;
    bool rc = getModel();
    if (rc)
    {
        rc = getMainFirmware() && getRADEFirmware();
        if (rc)
        {
            for (const auto &oneMount : m_MountList)
            {
                if (oneMount.model == m_FirmwareInfo.Model)
                {
                    // Make sure current mount firmware is larger
                    // than the minimum firmware required by this mount model.
                    if (m_FirmwareInfo.MainBoardFirmware >= oneMount.firmware)
                        return true;
                    else
                        LOGF_ERROR("Main board firmware is %s while minimum required firmware is %s. Please upgrade the mount firmware.",
                                   m_FirmwareInfo.MainBoardFirmware.c_str(), oneMount.firmware.c_str());
                }
            }

            return false;
        }
    }

    return rc;
}

bool Base::getModel()
{
    char res[DRIVER_LEN] = {0};

    // Do we support this mount?

    if (m_Simulation)
    {
        m_FirmwareInfo.Model = "GEM45";
        return true;
    }

    if (sendCommand(":MountInfo#", res, -1, 4))
    {
        std::string code = res;
        auto result = std::find_if(m_MountList.begin(), m_MountList.end(), [code](const MountInfo & oneMount)
        {
            return oneMount.code == code;
        });

        if (result != m_MountList.end())
        {
            m_FirmwareInfo.Model = result->model;
            return true;
        }

        LOGF_ERROR("Mount with code %s is not recognized.", res);
        return false;
    }

    return false;
}

bool Base::getMainFirmware()
{
    char res[DRIVER_LEN] = {0};

    if (m_Simulation)
    {
        m_FirmwareInfo.MainBoardFirmware = "191018";
        m_FirmwareInfo.ControllerFirmware = "191018";
        return true;
    }

    if (sendCommand(":FW1#", res))
    {
        char board[8] = {0}, controller[8] = {0};

        strncpy(board, res, 6);
        strncpy(controller, res + 6, 6);

        m_FirmwareInfo.MainBoardFirmware.assign(board, 6);
        m_FirmwareInfo.ControllerFirmware.assign(controller, 6);

        return true;
    }

    return false;
}

bool Base::getRADEFirmware()
{
    char res[DRIVER_LEN] = {0};

    if (m_Simulation)
    {
        m_FirmwareInfo.RAFirmware = "191018";
        m_FirmwareInfo.DEFirmware = "191018";
        return true;
    }

    if (sendCommand(":FW2#", res))
    {
        char ra[8] = {0}, de[8] = {0};

        strncpy(ra, res, 6);
        strncpy(de, res + 6, 6);

        m_FirmwareInfo.RAFirmware.assign(ra, 6);
        m_FirmwareInfo.DEFirmware.assign(de, 6);

        return true;
    }

    return false;
}

bool Base::startMotion(Direction dir)
{
    char cmd[DRIVER_LEN] = {0};
    switch (dir)
    {
        case IEQ_N:
            strcpy(cmd, ":mn#");
            break;
        case IEQ_S:
            strcpy(cmd, ":ms#");
            break;
        //        case IEQ_W:
        //            strcpy(cmd, ":mw#");
        //            break;
        //        case IEQ_E:
        //            strcpy(cmd, ":me#");
        //            break;
        // JM 2019-01-17: Appears iOptron implementation is reversed?
        case IEQ_W:
            strcpy(cmd, ":me#");
            break;
        case IEQ_E:
            strcpy(cmd, ":mw#");
            break;
    }

    return sendCommand(cmd);
}

bool Base::stopMotion(Direction dir)
{
    char cmd[DRIVER_LEN] = {0};
    char res[DRIVER_LEN] = {0};

    switch (dir)
    {
        case IEQ_N:
        case IEQ_S:
            strcpy(cmd, ":qD#");
            break;

        case IEQ_W:
        case IEQ_E:
            strcpy(cmd, ":qR#");
            break;
    }

    return sendCommand(cmd, res, -1, 1);
}

bool Base::findHome()
{
    if (!isCommandSupported("MSH"))
        return false;

    char res[DRIVER_LEN] = {0};

    return sendCommand(":MSH#", res, -1, 1);
}

bool Base::gotoHome()
{
    char res[DRIVER_LEN] = {0};
    return sendCommand(":MH#", res, -1, 1);
}

bool Base::setCurrentHome()
{
    char res[DRIVER_LEN] = {0};
    return sendCommand(":SZP#", res, -1, 1);
}

bool Base::setSlewRate(SlewRate rate)
{
    char cmd[DRIVER_LEN] = {0};
    char res[DRIVER_LEN] = {0};
    snprintf(cmd, DRIVER_LEN, ":SR%d#", (static_cast<int>(rate) + 1));
    return sendCommand(cmd, res, -1, 1);
}

bool Base::setTrackMode(TrackRate rate)
{
    char cmd[DRIVER_LEN] = {0};
    char res[DRIVER_LEN] = {0};

    switch (rate)
    {
        case TR_SIDEREAL:
            strcpy(cmd, ":RT0#");
            break;
        case TR_LUNAR:
            strcpy(cmd, ":RT1#");
            break;
        case TR_SOLAR:
            strcpy(cmd, ":RT2#");
            break;
        case TR_KING:
            strcpy(cmd, ":RT3#");
            break;
        case TR_CUSTOM:
            strcpy(cmd, ":RT4#");
            break;
    }

    return sendCommand(cmd, res, -1, 1);
}

bool Base::setCustomRATrackRate(double rate)
{
    if (!isCommandSupported("RR"))
        return false;

    // Limit to 0.5 to 1.5 as per docs
    rate = std::max(0.5, std::min(rate, 1.5));

    char cmd[DRIVER_LEN] = {0};
    char res[DRIVER_LEN] = {0};
    // Need to be in n.nnnn * sidereal_rate format.
    // e.g. 0.5 * 1e5 ==> 50000
    snprintf(cmd, DRIVER_LEN, ":RR%05d#", static_cast<int>(rate * 1e5));

    return sendCommand(cmd, res, -1, 1);
}

bool Base::setGuideRate(double raRate, double deRate)
{
    if (!isCommandSupported("RG"))
        return false;

    // Limit to 0.01 to 0.90 as per docs
    raRate = std::max(0.01, std::min(raRate, 0.9));
    // Limit to 0.10 to 0.99 as per docs
    deRate = std::max(0.1, std::min(deRate, 0.99));

    char cmd[DRIVER_LEN] = {0};
    char res[DRIVER_LEN] = {0};
    snprintf(cmd, DRIVER_LEN, ":RG%02d%02d#", static_cast<int>(raRate * 100.0), static_cast<int>(deRate * 100.0));

    return sendCommand(cmd, res, -1, 1);
}

bool Base::getGuideRate(double *raRate, double *deRate)
{
    if(m_Simulation)
    {
        *raRate = 0.5;
        *deRate = 0.5;
        return true;
    }
    
    if (!isCommandSupported("AG"))
        return false;

    char res[DRIVER_LEN] = {0};

    if (sendCommand(":AG#", res))
    {
        *raRate = DecodeString(res, 2, 100.0);
        *deRate = DecodeString(res + 2, 2, 100.0);

        return true;
    }

    return false;
}

bool Base::startGuide(Direction dir, uint32_t ms)
{
    char cmd[DRIVER_LEN] = {0};
    char dir_c = 0;

    switch (dir)
    {
        case IEQ_N:
            dir_c = 'n';
            break;

        case IEQ_S:
            dir_c = 's';
            break;

        case IEQ_W:
            dir_c = 'w';
            break;

        case IEQ_E:
            dir_c = 'e';
            break;
    }

    snprintf(cmd, DRIVER_LEN, ":M%c%05ud#", dir_c, ms);

    return sendCommand(cmd);
}

/* v3.0 Added in control for PEC , Train and Data Integrity */
bool Base::setPECEnabled(bool enabled)  
{ 
    return sendCommand(enabled ? ":SPP1#" : ":SPP0#");
}

bool Base::setPETEnabled(bool enabled)
{
    return sendCommand(enabled ? ":SPR1#" : ":SPR0#");
}

bool Base::getPETEnabled(bool enabled)
{
    char res[DRIVER_LEN] = {0};
    //  If enabled true then check data quality -> :GPE#
    //  If enabled false then check if training -> :GPR#
    if(enabled)
    {
        if (sendCommand(":GPE#",res))
         {
            if (res[0] == '1'){return true;}
         }
    }
    else
    {
         if (sendCommand(":GPR#",res))
         {
            if (res[0] == '1'){return true;}
         } 
    }      
    return false;
}
// End Mod */

bool Base::park()
{
    if (!isCommandSupported("MP1"))
        return false;

    char res[DRIVER_LEN] = {0};

    if (sendCommand(":MP1#", res, -1, 1))
    {
        return res[0] == '1';
    }

    return false;
}

bool Base::unpark()
{
    if (!isCommandSupported("MP0"))
        return false;

    char res[DRIVER_LEN] = {0};
    return sendCommand(":MP0#", res, -1, 1);
}

bool Base::abort()
{
    char res[DRIVER_LEN] = {0};
    return sendCommand(":Q#", res, -1, 1);
}

bool Base::slew()
{
    char res[DRIVER_LEN] = {0};

    if (sendCommand(":MS#", res, -1, 1))
    {
        return res[0] == '1';
    }

    return false;
}

bool Base::sync()
{
    char res[DRIVER_LEN] = {0};
    return sendCommand(":CM#", res, -1, 1);
}

bool Base::setTrackEnabled(bool enabled)
{
    char res[DRIVER_LEN] = {0};
    return sendCommand(enabled ? ":ST1#" : ":ST0#", res, -1, 1);
}

bool Base::setRA(double ra)
{
    char cmd[DRIVER_LEN] = {0};
    char res[DRIVER_LEN] = {0};

    // Send as milliseconds resolution
    int ieqValue = static_cast<int>(ra * 60 * 60 * 1000);

    snprintf(cmd, DRIVER_LEN, ":Sr%08d#", ieqValue);

    return sendCommand(cmd, res, -1, 1);
}

bool Base::setDE(double dec)
{
    char cmd[DRIVER_LEN] = {0};
    char res[DRIVER_LEN] = {0};

    // Send as 0.01 arcseconds resolution
    int ieqValue = static_cast<int>(fabs(dec) * 60 * 60 * 100);

    snprintf(cmd, DRIVER_LEN, ":Sd%c%08d#", (dec >= 0) ? '+' : '-', ieqValue);

    return sendCommand(cmd, res, -1, 1);
}

bool Base::setAz(double az)
{
    char cmd[DRIVER_LEN] = {0};
    char res[DRIVER_LEN] = {0};

    // Send as 0.01 arcsec resolution
    int ieqValue = static_cast<int>(az * 60 * 60 * 100);

    snprintf(cmd, DRIVER_LEN, ":Sz%09d#", ieqValue);

    return sendCommand(cmd, res, -1, 1);
}

bool Base::setAlt(double alt)
{
    char cmd[DRIVER_LEN] = {0};
    char res[DRIVER_LEN] = {0};

    // Send as 0.01 arcsec resolution
    int ieqValue = static_cast<int>(alt * 60 * 60 * 100);

    snprintf(cmd, DRIVER_LEN, ":Sa%c%08d#", (alt >= 0) ? '+' : '-', ieqValue);

    return sendCommand(cmd, res, -1, 1);
}

bool Base::setParkAz(double az)
{
    if (!isCommandSupported("SPA"))
        return false;

    char cmd[DRIVER_LEN] = {0};
    char res[DRIVER_LEN] = {0};

    // Send as 0.01 arcsec resolution
    int ieqValue = static_cast<int>(az * 60 * 60 * 100);

    snprintf(cmd, DRIVER_LEN, ":SPA%09d#", ieqValue);

    return sendCommand(cmd, res, -1, 1);
}

bool Base::setParkAlt(double alt)
{
    if (!isCommandSupported("SPH"))
        return false;

    char cmd[DRIVER_LEN] = {0};
    char res[DRIVER_LEN] = {0};

    alt = std::max(0.0, alt);

    // Send as 0.01 arcsec resolution
    int ieqValue = static_cast<int>(alt * 60 * 60 * 100);

    snprintf(cmd, DRIVER_LEN, ":SPH%08d#", ieqValue);

    return sendCommand(cmd, res, -1, 1);
}

bool Base::setLongitude(double longitude)
{
    char cmd[DRIVER_LEN] = {0};
    char res[DRIVER_LEN] = {0};

    int arcsecs = static_cast<int>(fabs(longitude) * 60 * 60);
    snprintf(cmd, DRIVER_LEN, ":Sg%c%06d#", (longitude >= 0) ? '+' : '-', arcsecs);

    return sendCommand(cmd, res, -1, 1);
}

bool Base::setLatitude(double latitude)
{
    char cmd[DRIVER_LEN] = {0};
    char res[DRIVER_LEN] = {0};

    int arcsecs = static_cast<int>(fabs(latitude) * 60 * 60);
    snprintf(cmd, DRIVER_LEN, ":St%c%06d#", (latitude >= 0) ? '+' : '-', arcsecs);

    return sendCommand(cmd, res, -1, 1);
}

bool Base::setLocalDate(int yy, int mm, int dd)
{
    char cmd[DRIVER_LEN] = {0};
    char res[DRIVER_LEN] = {0};

    snprintf(cmd, DRIVER_LEN, ":SC%02d%02d%02d#", yy, mm, dd);

    return sendCommand(cmd, res, -1, 1);
}

bool Base::setLocalTime(int hh, int mm, int ss)
{
    char cmd[DRIVER_LEN] = {0};
    char res[DRIVER_LEN] = {0};

    snprintf(cmd, DRIVER_LEN, ":SL%02d%02d%02d#", hh, mm, ss);

    return sendCommand(cmd, res, -1, 1);
}

bool Base::setDST(bool enabled)
{
    char res[DRIVER_LEN] = {0};
    return sendCommand(enabled ? ":SDS1#" : ":SDS0#", res, -1, 1);
}

bool Base::setUTCOffset(double offset_hours)
{
    char cmd[DRIVER_LEN] = {0};
    char res[DRIVER_LEN] = {0};

    int offset_minutes = static_cast<int>(fabs(offset_hours) * 60.0);
    snprintf(cmd, 16, ":SG%c%03d#", (offset_hours >= 0) ? '+' : '-', offset_minutes);

    return sendCommand(cmd, res, -1, 1);
}

bool Base::getCoords(double *ra, double *dec)
{
    char res[DRIVER_LEN] = {0};

    if (sendCommand(":GEC#", res))
    {
        *ra = Ra = DecodeString(res + 9, 8, ieqHours);
        *dec = Dec = DecodeString(res, 9, ieqDegrees);
        return true;
    }

    return false;
}

bool Base::getUTCDateTime(double *utc_hours, int *yy, int *mm, int *dd, int *hh, int *minute, int *ss)
{
    char res[DRIVER_LEN] = {0};

    if (sendCommand(":GLT#", res))
    {
        *utc_hours = DecodeString(res, 4, 60.0);
        *yy = DecodeString(res + 5, 2) + 2000;
        *mm = DecodeString(res + 7, 2);
        *dd = DecodeString(res + 9, 2);
        *hh = DecodeString(res + 11, 2);
        *minute = DecodeString(res + 13, 2);
        *ss = DecodeString(res + 15, 2);

        ln_zonedate localTime;
        ln_date utcTime;

        localTime.years   = *yy;
        localTime.months  = *mm;
        localTime.days    = *dd;
        localTime.hours   = *hh;
        localTime.minutes = *minute;
        localTime.seconds = *ss;
        localTime.gmtoff  = static_cast<long>(*utc_hours * 3600);

        ln_zonedate_to_date(&localTime, &utcTime);

        *yy     = utcTime.years;
        *mm     = utcTime.months;
        *dd     = utcTime.days;
        *hh     = utcTime.hours;
        *minute = utcTime.minutes;
        *ss     = static_cast<int>(utcTime.seconds);

        return true;

    }

    return false;
}

bool Base::getStatus(Info *info)
{
    char res[DRIVER_LEN] = {0};

    if (sendCommand(":GLS#", res))
    {
        char longitude[8] = {0}, latitude[8] = {0}, status[8] = {0};

        strncpy(longitude, res, 7);
        strncpy(latitude, res + 7, 6);
        strncpy(status, res + 13, 6);

        info->longitude     = DecodeString(res, 7, 3600.0);
        info->latitude      = DecodeString(res + 7, 6, 3600.0) - 90;
        info->gpsStatus     = static_cast<GPSStatus>(status[0] - '0');
        info->systemStatus  = static_cast<SystemStatus>(status[1] - '0');
        info->trackRate     = static_cast<TrackRate>(status[2] - '0');
        info->slewRate      = static_cast<SlewRate>(status[3] - '0' - 1);
        info->timeSource    = static_cast<TimeSource>(status[4] - '0');
        info->hemisphere    = static_cast<Hemisphere>(status[5] - '0');

        this->info = *info;     // keep a local copy

        return true;
    }

    return false;
}

const char * pierSideStr(IEQ_PIER_SIDE ps)
{
    switch (ps)
    {
        case IEQ_PIER_EAST:
            return "EAST";
        case IEQ_PIER_WEST:
            return "WEST";
        case IEQ_PIER_UNKNOWN:
            return "UNKNOWN";
        case IEQ_PIER_UNCERTAIN:
            return "UNCERTAIN";
    }
    return "Impossible";
}

bool Base::getPierSide(IEQ_PIER_SIDE * pierSide)
{
    char res[DRIVER_LEN] = {0};

    // use the GEA command, hoping that it returns the dec and polar angle axis positions
    // see https://www.indilib.org/forum/mounts/6720-ioptron-cem60-question.html#52154
    // the polar angle is in units of 1/100 arc second, signed, and the hour angle is in milliseconds,
    // possibly with an offset. The home axis positions are PA +0.0, HA 12.0.
    // For the West pointing state the ha = 18 - haAxis.
    //
    if (sendCommand(":GEA#", res))
    {
        // get the hour angle in hours
        haAxis  = DecodeString(res + 9, 8, ieqHours);
        // this is the pole angle in degrees
        decAxis = DecodeString(res, 9, ieqDegrees);

        double axisHa = 0;

        if (decAxis >= 0)
        {
            *pierSide = IEQ_PIER_WEST;
            axisHa  = 18 - haAxis;        // OK for the West PS
        }
        else
        {
            *pierSide = IEQ_PIER_EAST;
            axisHa  = haAxis - 6;        // OK for the West PS
        }

        // The pole angle is not exactly at 0 when the dec is 90 and this gives problems with incorrect pier side close to the pole.
        //
        // Attempt to handle this by using the hour angle where the HA can be relied on - away from the meridian
        // Use pole angle when within 2 hours of a meridian.
        // If the pole angle is less than the difference between the pole angle and the dec report the pier side as unknown
        //
        // I know, horrible, but the data the mount reports is so difficult to interpret that this seems to be the least
        // worst solution, anyway, let's see if it works CR

        double lst = get_local_sidereal_time(info.longitude);
        double ha = rangeHA(get_local_hour_angle(lst, Ra));

        const char* reason;
        double decPA = info.latitude >= 0 ? 90 - Dec : 90 +
                       Dec;     // the distance from the pole determined using the declination, ok for both hemispheres

        if ((ha > 2 && ha < 10) || (ha < -2 && ha > -10))
        {
            // use Ha to determine pier side
            *pierSide = ha > 0 ? IEQ_PIER_EAST : IEQ_PIER_WEST;
            reason = "Hour Angle";
        }
        else
        {
            double decDiff = std::fabs(decPA - std::fabs(decAxis)); // not sure about this in the Southern hemisphere
            if (decPA > decDiff)
            {
                // use the pole angle
                *pierSide = decAxis > 0 ? IEQ_PIER_WEST : IEQ_PIER_EAST;
                reason = "pole angle";
            }
            else
            {
                *pierSide = IEQ_PIER_UNCERTAIN;
                reason = "uncertain";
            }
        }

        LOGF_DEBUG("getPierSide pole Axis %f, haAxis %f, axisHa %f, ha %f, decPa %f, %s pierSide %s", decAxis, haAxis, axisHa, ha,
                   decPA, reason,
                   pierSideStr(*pierSide));

        return true;
    }
    *pierSide = IEQ_PIER_UNKNOWN;
    return false;
}

bool Base::sendCommand(const char * cmd, char * res, int cmd_len, int res_len)
{
    int nbytes_written = 0, nbytes_read = 0, rc = -1;

    if (m_Simulation)
        return true;

    tcflush(m_PortFD, TCIOFLUSH);

    if (cmd_len > 0)
    {
        char hex_cmd[DRIVER_LEN * 3] = {0};
        hexDump(hex_cmd, cmd, cmd_len);
        LOGF_DEBUG("CMD <%s>", hex_cmd);
        rc = tty_write(m_PortFD, cmd, cmd_len, &nbytes_written);
    }
    else
    {
        LOGF_DEBUG("CMD <%s>", cmd);
        rc = tty_write_string(m_PortFD, cmd, &nbytes_written);
    }

    if (rc != TTY_OK)
    {
        char errstr[MAXRBUF] = {0};
        tty_error_msg(rc, errstr, MAXRBUF);
        LOGF_ERROR("Serial write error: %s.", errstr);
        return false;
    }

    if (res == nullptr)
        return true;

    if (res_len > 0)
        rc = tty_read(m_PortFD, res, res_len, DRIVER_TIMEOUT, &nbytes_read);
    else
        rc = tty_nread_section(m_PortFD, res, DRIVER_LEN, DRIVER_STOP_CHAR, DRIVER_TIMEOUT, &nbytes_read);

    if (rc != TTY_OK)
    {
        char errstr[MAXRBUF] = {0};
        tty_error_msg(rc, errstr, MAXRBUF);
        LOGF_ERROR("Serial read error: %s.", errstr);
        return false;
    }

    if (res_len > 0)
    {
        char hex_res[DRIVER_LEN * 3] = {0};
        hexDump(hex_res, res, res_len);
        LOGF_DEBUG("RES <%s>", hex_res);
    }
    else
    {
        LOGF_DEBUG("RES <%s>", res);
    }

    tcflush(m_PortFD, TCIOFLUSH);

    return true;
}

void Base::hexDump(char * buf, const char * data, int size)
{
    for (int i = 0; i < size; i++)
        sprintf(buf + 3 * i, "%02X ", static_cast<uint8_t>(data[i]));

    if (size > 0)
        buf[3 * size - 1] = '\0';
}

bool Base::isCommandSupported(const std::string &command, bool silent)
{
    // Find Home
    if (command == "MSH")
    {
        if (m_FirmwareInfo.Model.find("CEM60") == std::string::npos &&
                m_FirmwareInfo.Model.find("CEM40") == std::string::npos &&
                m_FirmwareInfo.Model.find("GEM45") == std::string::npos)
        {
            if (!silent)
                LOG_ERROR("Finding home is only supported on CEM40, GEM45 and CEM60 mounts.");
            return false;

        }
    }
    else if (command == "RR")
    {
        if (m_FirmwareInfo.Model.find("AA") != std::string::npos)
        {
            if (!silent)
                LOG_ERROR("Tracking rate is not supported on Altitude-Azimuth mounts.");
            return false;
        }
    }
    else if (command == "RG" || command == "AG")
    {
        if (m_FirmwareInfo.Model.find("AA") != std::string::npos)
        {
            if (!silent)
                LOG_ERROR("Guide rate is not supported on Altitude-Azimuth mounts.");
            return false;
        }
    }
    if (command == "MP0" || command == "MP1" || command == "SPA" || command == "SPH")
    {
        if (m_FirmwareInfo.Model.find("CEM60") == std::string::npos &&
                m_FirmwareInfo.Model.find("CEM40") == std::string::npos &&
                m_FirmwareInfo.Model.find("GEM45") == std::string::npos &&
                m_FirmwareInfo.Model.find("iEQ") == std::string::npos)
        {
            if (!silent)
                LOG_ERROR("Parking only supported on CEM40, GEM45, CEM60, iEQPro 30 and iEQ Pro 45.");
            return false;
        }
    }

    return true;
}

double Base::DecodeString(const char * data, size_t size, double factor)
{
    return DecodeString(data, size) / factor;
}

int Base::DecodeString(const char *data, size_t size)
{
    char str[DRIVER_LEN / 2] = {0};
    strncpy(str, data, size);

    int iVal = atoi(str);
    return iVal;
}

void Base::setSimulation(bool enable)
{
    m_Simulation = enable;

    simData.ra_guide_rate = 0.5;
    simData.de_guide_rate = 0.5;
    simData.pier_state = IEQ_PIER_WEST;
    simData.JD = ln_get_julian_from_sys();
    simData.utc_offset_minutes = 3 * 60;
    simData.day_light_saving = false;

    simData.simInfo.gpsStatus = GPS_DATA_OK;
    simData.simInfo.hemisphere = HEMI_NORTH;
    simData.simInfo.slewRate = SR_6;
    simData.simInfo.timeSource = TS_GPS;
    simData.simInfo.trackRate = TR_SIDEREAL;
    simData.simInfo.longitude = 48.1;
    simData.simInfo.latitude  = 29.5;
}

void Base::setSimGPSstatus(GPSStatus value)
{
    simData.simInfo.gpsStatus = value;
}

void Base::setSimSytemStatus(SystemStatus value)
{
    simData.simInfo.systemStatus = value;
}

void Base::setSimTrackRate(TrackRate value)
{
    simData.simInfo.trackRate = value;
}

void Base::setSimSlewRate(SlewRate value)
{
    simData.simInfo.slewRate = value;
}

void Base::setSimTimeSource(TimeSource value)
{
    simData.simInfo.timeSource = value;
}

void Base::setSimHemisphere(Hemisphere value)
{
    simData.simInfo.hemisphere = value;
}

void Base::setSimRA(double ra)
{
    simData.ra = ra;
}

void Base::setSimDE(double de)
{
    simData.de = de;
}

void Base::setSimGuideRate(double raRate, double deRate)
{
    simData.ra_guide_rate = raRate;
    simData.de_guide_rate = deRate;
}

void Base::setSimLongLat(double longitude, double latitude)
{
    simData.simInfo.longitude = longitude;
    simData.simInfo.latitude  = latitude;
}


}
