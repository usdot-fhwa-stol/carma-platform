#pragma once
#include "delphi_types.h"

#include <cav_driver_utils/can/can_interface.h>
#include <cav_driver_utils/synchronized_async_process_queue.h>
#include <boost/signals2/signal.hpp>
#include <cav_driver_utils/can/error_code.h>


template <typename T,typename = typename std::enable_if<std::is_base_of<cav::CANInterface, T>::value>::type >
class DelphiESRCANClient;

template<class T>
class DelphiESRCANClient<T>
{
public:

    /**
     * @brief Data structure to whole all of one burst coming from the sensor
     */
    struct BurstData
    {
        boost::posix_time::ptime stamp;
        delphi::ESRStatus1 esrStatus1;
        delphi::ESRStatus2 esrStatus2;
        delphi::ESRStatus3 esrStatus3;
        delphi::ESRStatus4 esrStatus4;
        delphi::ESRStatus5 esrStatus5;
        delphi::ESRStatus6 esrStatus6;
        delphi::ESRStatus7 esrStatus7;
        delphi::ESRStatus8 esrStatus8;
        delphi::ESRStatus9 esrStatus9;
        std::vector<delphi::ESRTrack> tracks;
    };

private:

    std::shared_ptr<T> device;
    typedef cav::SynchronizedAsyncProcessQueue<std::shared_ptr<cav::CANFrameStamped const>> _sync_q_type;
    std::unique_ptr<_sync_q_type> process_q_;
    BurstData burst_data;

    /**
     * @brief message definitions used by this class to parse CAN messages
     */
    struct
    {
        delphi::ESRStatus1Definition esr1status;
        delphi::ESRStatus2Definition esr2status;
        delphi::ESRStatus3Definition esr3status;
        delphi::ESRStatus4Definition esr4status;
        delphi::ESRStatus5Definition esr5status;
        delphi::ESRStatus6Definition esr6status;
        delphi::ESRStatus7Definition esr7status;
        delphi::ESRStatus8Definition esr8status;
        delphi::ESRStatus9Definition esr9status;
        delphi::ESRTrackDefinition trackDefinition;
        delphi::ESRTrackMotionPowerDefinition trackPowerDefinition;

    }message_definitions;

    template <typename K>
    constexpr K can_bit(const K& n)
    {
        return (8*(7-n/8)+n%8);
    }

    template <typename K>
    constexpr K can_mask(const K& n)
    {
        return ((1<<n)-1);
    }

public:

    /**
     * @brief Signal called on receipt of a complete burst from sensor
     */
    boost::signals2::signal<void (const BurstData&)> onBurstReceived;

    /**
     * @brief Signal called when device sends an error
     */
    boost::signals2::signal<void (const boost::system::error_code&)> onError;

    /**
     * @brief Signal called when device sends CAN error
     */
    boost::signals2::signal<void (const cav::can::ErrorCode_t&)> onCANError;


    /**
     * @brief Constructs a DelphiESRCanClient
     * @param device - shared_ptr to a device that implements cav::CANInterface
     */
    DelphiESRCANClient(std::shared_ptr<T>& device) :
            device(device),
            process_q_(new _sync_q_type(std::bind(&DelphiESRCANClient::process,this,std::placeholders::_1)))
    {
    }

    DelphiESRCANClient(const DelphiESRCANClient&) = delete;
    DelphiESRCANClient(DelphiESRCANClient&&) = delete;

    ~DelphiESRCANClient()
    {
        device.reset();
        process_q_.reset();
    }

    /**
     * @brief Initializes the device and begins listening for frames
     */
    void init()
    {
        burst_data.tracks.resize(64);
        device->setFilters(can_ids);
        device->onFrameReceived.connect(std::bind(&DelphiESRCANClient::frameReceivedHandler,this,std::placeholders::_1));
        device->onErrorFrameReceived.connect(std::bind(&DelphiESRCANClient::errorFrameReceivedHandler,this,std::placeholders::_1));
        device->onError.connect(std::bind(&DelphiESRCANClient::errorReceivedHandler,this,std::placeholders::_1));
    }

    /**
     * @brief Send commands to the sensor
     * @param rx0
     * @param rx1
     */
    void sendCommands(const delphi::DelphiRX4F0& rx0, const delphi::DelphiRX4F1& rx1)
    {
        //rx0
        {
            uint64_t data = 0;
            uint16_t veh_speed = (uint16_t)(rx0.speed/0.0625);
            data |= ((uint64_t)(veh_speed&can_mask<uint64_t>(11)) << can_bit(13));

            if(rx0.speed_direction == delphi::SpeedDirection::Reverse)
                data |= ((uint64_t)(0x01) << can_bit(12));

            int16_t yaw_rate = (int16_t)(rx0.yaw_rate/0.0625);
            data |= ((uint64_t)(yaw_rate&can_mask<uint64_t>(12)) << can_bit(16));

            data |= ((uint64_t)(0x01) << can_bit(31));

            data |= ((uint64_t)(rx0.radius_curvature & can_mask<uint64_t>(14)) << can_bit(32));

            data |= ((uint64_t)(rx0.steering_wheel_angle)&can_mask<uint64_t>(11) << can_bit(51));

            if(rx0.steering_wheel_sign == delphi::SteeringWheelSign::Clockwise)
                data |= ((uint64_t)(0x01) << can_bit(46));

            data |= ((uint64_t)(rx0.steering_wheel_angle_rate)&can_mask<uint64_t>(11)<<can_bit(56));

            if(rx0.steering_wheel_angle_rate_sign == delphi::SteeringWheelSign::Clockwise)
                data |= ((uint64_t)(0x01) << can_bit(30));

            if(rx0.steering_angle_validity == delphi::Validity::AvailableAndValid)
                data |= ((uint64_t)0x01 << can_bit(47));

            cav::CANFrameStamped cf;
            cf.id = 0x4f0;
            cf.is_rtr = false;
            cf.is_extended = false;
            cf.is_error = false;
            cf.setData(data);
            cf.dlc = 8;
            device->write(cf);
        }

        //rx1
        {
            uint64_t data = 0;
            data |= ((uint64_t)(rx1.scan_index_ack) << can_bit(8));

            int8_t lat_mount_offset = (int8_t)(rx1.alignment_angle_offset/0.015625);
            data |= ((uint64_t)(lat_mount_offset) << can_bit(40));

            int8_t angle_misalign = (int8_t)(rx1.angle_misalignment/0.0625);
            data |= ((uint64_t)(angle_misalign) << can_bit(32));

            data |= ((uint64_t)((rx1.maximum_tracks)&can_mask(6))<<can_bit(48));

            if(rx1.radiate_command == delphi::SensorModeCommand::Radiate)
                data |= ((uint64_t)(0x01)<<can_bit(55));

            if(rx1.wiper_status == delphi::WindShieldWiperStatus::On)
                data |= ((uint64_t)(0x01)<<can_bit(57));

            if(rx1.raw_data_enable == delphi::RawDataEnable::Raw)
                data |= ((uint64_t)(0x01)<<can_bit(56));

            data |= ((static_cast<uint64_t>(rx1.grouping_mode) & can_mask<uint64_t>(2)) << can_bit(58));

            if(rx1.mmr_upside_down)
                data |= ((uint64_t)(0x01) << can_bit(60));

            if(rx1.vehicle_speed_validity == delphi::Validity::AvailableAndValid)
                data |= ((uint64_t)(0x01) << can_bit(60));

            data |= ((static_cast<uint64_t >(rx1.turn_signal_status)&can_mask<uint64_t>(2)) << can_bit(62));

            if(rx1.blockage_disable)
                data |= ((uint64_t)(0x01) << can_bit(54));

            if(rx1.use_angle_misalignment)
                data |= ((uint64_t)(0x01) << can_bit(23));

            if(rx1.clear_faults)
                data |= ((uint64_t)(0x01) << can_bit(22));

            if(rx1.lr_only_trasmit)
                data |= ((uint64_t)(0x01) << can_bit(24));

            if(rx1.mr_only_transmit)
                data |= ((uint64_t)(0x01) << can_bit(25));

            data |= ((uint64_t)(rx1.high_yaw_angle&can_mask<uint64_t>(6)) << can_bit(16));

            int8_t track_roc = (int8_t)(rx1.short_track_roc/500);
            data |= ((uint64_t)(track_roc&can_mask(4))<<can_bit(28));

            cav::CANFrameStamped cf;
            cf.id = 0x4f1;
            cf.is_rtr = false;
            cf.is_extended = false;
            cf.is_error = false;
            cf.setData(data);
            cf.dlc = 8;
            device->write(cf);
        }
    }

private:

    /**
     * @brief Pertinent CAN Ids on the bus
     */
    std::vector<uint16_t> can_ids =
            {
                    0x4E0,
                    0x4E1,
                    0x4E2,
                    0x4E3,
                    0x500,
                    0x501,
                    0x502,
                    0x503,
                    0x504,
                    0x505,
                    0x506,
                    0x507,
                    0x508,
                    0x509,
                    0x50A,
                    0x50B,
                    0x50C,
                    0x50D,
                    0x50E,
                    0x50F,
                    0x510,
                    0x511,
                    0x512,
                    0x513,
                    0x514,
                    0x515,
                    0x516,
                    0x517,
                    0x518,
                    0x519,
                    0x51A,
                    0x51B,
                    0x51C,
                    0x51D,
                    0x51E,
                    0x51F,
                    0x520,
                    0x521,
                    0x522,
                    0x523,
                    0x524,
                    0x525,
                    0x526,
                    0x527,
                    0x528,
                    0x529,
                    0x52A,
                    0x52B,
                    0x52C,
                    0x52D,
                    0x52E,
                    0x52F,
                    0x530,
                    0x531,
                    0x532,
                    0x533,
                    0x534,
                    0x535,
                    0x536,
                    0x537,
                    0x538,
                    0x539,
                    0x53A,
                    0x53B,
                    0x53C,
                    0x53D,
                    0x53E,
                    0x53F,
                    0x540,
                    0x5E4,
                    0x5E5,
                    0x5E6,
                    0x5E7,
                    0x5E8,
            };


    /**
     * @brief Handles a frame received. The frame is queued into an asynchronous process queue to avoid
     * congesting the receive thread and to separate the work
     * @param msg
     */
    void frameReceivedHandler(std::shared_ptr<cav::CANFrameStamped const> msg)
    {
        process_q_->push(msg);
    }

    /**
     * @brief Handles an error frame received. The frame is queued into an asynchronous process queue to avoid
     * congesting the receive thread and to separate the work
     * @param msg
     */
    void errorFrameReceivedHandler(std::shared_ptr<cav::CANFrameStamped const> msg)
    {
        process_q_->push(msg);
    }

    /**
     * @brief Helper function to check if the can_id received is the expected can_id . maintains a state
     * of the previous can_id called with
     * @param can_id
     * @return
     */
    bool isExpectedID(uint32_t can_id)
    {
        static uint32_t last_id = 0;
        static bool in_sync = true;
        if( 0x4E0 <= can_id && 0x5E8 >= can_id && !(0x5B0 <= can_id && 0x5BE >= can_id) )
        {
            if( 0x4E0 == can_id )
            {
                in_sync = true;
            }
            else if( 0x500 == can_id )
            {
                if( last_id != 0x4E3 )
                {
                    in_sync = false;
                }
            }
            else if( 0x540 == can_id )
            {
                if( last_id != 0x53F && last_id != 0x540 )
                {
                    in_sync = false;
                }
            }
            else if( 0x5D0 == can_id )
            {
                if( last_id != 0x540 )
                {
                    in_sync = false;
                }
            }
            else if( 0x5E4 == can_id )
            {
                if( last_id != 0x540 && last_id != 0x5D1 )
                {
                    in_sync = false;
                }
            }
            else
            {
                if( last_id != (can_id-1) )
                {
                    in_sync = false;
                }
            }
        }

        last_id = can_id;
        return in_sync;
    }

    /**
     * @brief processes can messages, called by the async process queue
     * @param msg
     */
    void process(const std::shared_ptr<cav::CANFrameStamped const> msg)
    {
        //Is this message an error?
        if(msg->is_error)
        {
            cav::can::ErrorCode_t ec;
            ec.code = msg->id;
            onCANError(ec);
            return;
        }

        static bool invalid_data = false;
        //if we have invalid data we only wish to continue once we get back to the first message 0x4e0
        if(invalid_data && msg->id != 0x4E0){return;}

        switch(msg->id)
        {
            case 0x4E0: //status 1
                burst_data.stamp = msg->stamp;
                invalid_data = !isExpectedID(msg->id);
                message_definitions.esr1status.setData(&msg->data[0],msg->dlc);
                burst_data.esrStatus1.setFromDefinition(message_definitions.esr1status);
                break;
            case 0x4E1://status 2
                invalid_data = !isExpectedID(msg->id);
                message_definitions.esr2status.setData(&msg->data[0],msg->dlc);
                burst_data.esrStatus2.setFromDefinition(message_definitions.esr2status);
                break;
            case 0x4E2://status 3
                invalid_data = !isExpectedID(msg->id);
                message_definitions.esr3status.setData(&msg->data[0],msg->dlc);
                burst_data.esrStatus3.setFromDefinition(message_definitions.esr3status);
                break;
            case 0x4E3://status 4
                invalid_data = !isExpectedID(msg->id);
                message_definitions.esr4status.setData(&msg->data[0],msg->dlc);
                burst_data.esrStatus4.setFromDefinition(message_definitions.esr4status);
                break;
            case 0x500: //beginning of tracks
            case 0x501:
            case 0x502:
            case 0x503:
            case 0x504:
            case 0x505:
            case 0x506:
            case 0x507:
            case 0x508:
            case 0x509:
            case 0x50A:
            case 0x50B:
            case 0x50C:
            case 0x50D:
            case 0x50E:
            case 0x50F:
            case 0x510:
            case 0x511:
            case 0x512:
            case 0x513:
            case 0x514:
            case 0x515:
            case 0x516:
            case 0x517:
            case 0x518:
            case 0x519:
            case 0x51A:
            case 0x51B:
            case 0x51C:
            case 0x51D:
            case 0x51E:
            case 0x51F:
            case 0x520:
            case 0x521:
            case 0x522:
            case 0x523:
            case 0x524:
            case 0x525:
            case 0x526:
            case 0x527:
            case 0x528:
            case 0x529:
            case 0x52A:
            case 0x52B:
            case 0x52C:
            case 0x52D:
            case 0x52E:
            case 0x52F:
            case 0x530:
            case 0x531:
            case 0x532:
            case 0x533:
            case 0x534:
            case 0x535:
            case 0x536:
            case 0x537:
            case 0x538:
            case 0x539:
            case 0x53A:
            case 0x53B:
            case 0x53C:
            case 0x53D:
            case 0x53E:
            case 0x53F: //end of tracks
                {
                    invalid_data = !isExpectedID(msg->id);
                    message_definitions.trackDefinition.setData(&msg->data[0],msg->dlc);
                    size_t idx = msg->id - 0x500;
                    if(idx < burst_data.tracks.size())
                        burst_data.tracks[idx].setFromDefinition(message_definitions.trackDefinition);
                }
                break;
            case 0x540: //track power
                {
                    invalid_data = !isExpectedID(msg->id);
                    message_definitions.trackPowerDefinition.setData(&msg->data[0],msg->dlc);
                    uint32_t group = message_definitions.trackPowerDefinition.group();
                    size_t stride = message_definitions.trackPowerDefinition.track().size();
                    for(size_t i = 0; i < message_definitions.trackPowerDefinition.track().size(); i++)
                    {
                        if(group*stride + i < burst_data.tracks.size())
                            burst_data.tracks[group*stride+i].setMovePower(message_definitions.trackPowerDefinition.track()[i]);
                    }
                }
                break;
            case 0x5E4: //status 5
                invalid_data = !isExpectedID(msg->id);
                message_definitions.esr5status.setData(&msg->data[0],msg->dlc);
                burst_data.esrStatus5.setFromDefinition(message_definitions.esr5status);
                break;
            case 0x5E5: //status 6
                invalid_data = !isExpectedID(msg->id);
                message_definitions.esr6status.setData(&msg->data[0],msg->dlc);
                burst_data.esrStatus6.setFromDefinition(message_definitions.esr6status);
                break;
            case 0x5E6: //status 7
                invalid_data = !isExpectedID(msg->id);
                message_definitions.esr7status.setData(&msg->data[0],msg->dlc);
                burst_data.esrStatus7.setFromDefinition(message_definitions.esr7status);
                break;
            case 0x5E7://status 8
                invalid_data = !isExpectedID(msg->id);
                message_definitions.esr8status.setData(&msg->data[0],msg->dlc);
                burst_data.esrStatus8.setFromDefinition(message_definitions.esr8status);
                break;
            case 0x5E8://status 9
                invalid_data = !isExpectedID(msg->id);
                message_definitions.esr9status.setData(&msg->data[0],msg->dlc);
                burst_data.esrStatus9.setFromDefinition(message_definitions.esr9status);
                if(!invalid_data)
                    onBurstReceived(burst_data);
                break;
            default:
                break;
        }
    }

    /**
     * @brief handler for errors signaled by device
     * @param ec
     */
    void errorReceivedHandler(const boost::system::error_code &ec)
    {
        onError(ec);
    }


};