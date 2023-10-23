/* 
 * Copyright (c) [year], [author]
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: [Your Name] <[Your Email]>
 */

#include "levy-flight-2d-mobility-model.h"

#include "ns3/double.h"
#include "ns3/enum.h"
#include "ns3/log.h"
#include "ns3/pointer.h"
#include "ns3/simulator.h"
#include "ns3/string.h"

#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("LevyFlight2d");
NS_OBJECT_ENSURE_REGISTERED(LevyFlight2dMobilityModel);

TypeId LevyFlight2dMobilityModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::LevyFlight2dMobilityModel")
            .SetParent<MobilityModel>()
            .SetGroupName("Mobility")
            .AddConstructor<LevyFlight2dMobilityModel>()
            .AddAttribute("Time",
                          "Change current direction and speed after moving for this delay.",
                          TimeValue(Seconds(1.0)),
                          MakeTimeAccessor(&LevyFlight2dMobilityModel::m_modeTime),
                          MakeTimeChecker())
            .AddAttribute("Direction",
                          "A random variable used to pick the direction (radians).",
                          StringValue("ns3::UniformRandomVariable[Min=0.0|Max=6.283184]"),
                          MakePointerAccessor(&LevyFlight2dMobilityModel::m_direction),
                          MakePointerChecker<RandomVariableStream>())
            .AddAttribute("Speed",
                          "A random variable used to pick the speed (m/s).",
                          StringValue("ns3::UniformRandomVariable[Min=2.0|Max=4.0]"),
                          MakePointerAccessor(&LevyFlight2dMobilityModel::m_speed),
                          MakePointerChecker<RandomVariableStream>())
            .AddAttribute("StepSize",
                          "A random variable used to pick the speed (m/s).",
                          StringValue("ns3::ParetoRandomVariable"),
                          MakePointerAccessor(&LevyFlight2dMobilityModel::m_stepSize),
                          MakePointerChecker<RandomVariableStream>());
    return tid;
}

void LevyFlight2dMobilityModel::DoInitialize()
{
    DoInitializePrivate();
    MobilityModel::DoInitialize();
}

void LevyFlight2dMobilityModel::DoInitializePrivate()
{
    m_helper.Update();
    Vector position = m_helper.GetCurrentPosition();

    double speed = m_speed->GetValue();
    double direction = m_direction->GetValue();

    Vector vector(std::cos(direction) * speed, std::sin(direction) * speed, 0.0);
    m_helper.SetVelocity(vector);
    m_helper.Unpause();

    double stepLength = m_stepSize->GetValue();

    Time delayLeft = m_modeTime;

    DoWalk(delayLeft);
}

void
LevyFlight2dMobilityModel::DoWalk(Time delayLeft)
{
    Vector position = m_helper.GetCurrentPosition();
    Vector speed = m_helper.GetVelocity();
    Vector nextPosition = position;
    nextPosition.x += speed.x * m_stepSize->GetValue() * delayLeft.GetSeconds();
    nextPosition.y += speed.y * m_stepSize->GetValue() * delayLeft.GetSeconds();

    m_event.Cancel();
            m_event =
            Simulator::Schedule(delayLeft, &LevyFlight2dMobilityModel::DoInitializePrivate, this);
    NotifyCourseChange();
}

void
LevyFlight2dMobilityModel::Rebound(Time delayLeft)
{
    m_helper.UpdateWithBounds(m_bounds);
    Vector position = m_helper.GetCurrentPosition();
    Vector speed = m_helper.GetVelocity();
    switch (m_bounds.GetClosestSide(position))
    {
    case Rectangle::RIGHT:
    case Rectangle::LEFT:
        speed.x = -speed.x;
        break;
    case Rectangle::TOP:
    case Rectangle::BOTTOM:
        speed.y = -speed.y;
        break;
    }
    m_helper.SetVelocity(speed);
    m_helper.Unpause();
    DoWalk(delayLeft);
}

void LevyFlight2dMobilityModel::DoDispose()
{
    // chain up
    MobilityModel::DoDispose();
}

Vector LevyFlight2dMobilityModel::DoGetPosition() const
{
    return m_helper.GetCurrentPosition();
}

void LevyFlight2dMobilityModel::DoSetPosition(const Vector &position)
{
    m_helper.SetPosition(position);
    m_event.Cancel();
    m_event = Simulator::ScheduleNow(&LevyFlight2dMobilityModel::DoInitializePrivate, this);
}

Vector LevyFlight2dMobilityModel::DoGetVelocity() const
{
    return m_helper.GetVelocity();
}

int64_t LevyFlight2dMobilityModel::DoAssignStreams(int64_t stream)
{
    m_speed->SetStream(stream);
    m_direction->SetStream(stream + 1);
    m_stepSize->SetStream(stream + 2);
    return 3;
}

} // namespace ns3

