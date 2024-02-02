//! **Lane Keep Assist (LKA)** – a warning that is provided automatically by the
//! vehicle in response to the vehicle that is about to drift beyond a delineated edge line of
//! the current travel lane

#![cfg_attr(not(test), no_std)]
#![deny(warnings, unsafe_code, missing_docs, clippy::all)]
use line_processing::{LkaLane, LkaLine};
use log::error;
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use num_traits::Float;

pub mod line_processing;
pub mod lka_inhibition;
pub mod lka_state;
use adas_db::{calibrations::Calibration, datatypes::*};
pub use lka_state::StateMachine;
use ls_common::{
    algo::virtual_line::{Crossed, VirtualLines},
    inhibition::CommonInhibition,
    io::*,
    state::*,
};

/// Main structure to represent LKA feature
#[derive(Debug, Clone)]
pub struct Lka {
    /// state
    pub state: StateMachine,
    /// state for lkw alert similar to LDW
    pub lkw_state: StateMachine,
    /// Lane
    pub lane: Lane,
    /// lkw lane
    pub lkw_lane: Lane,
    /// inhibition
    pub inhibition: CommonInhibition,
    /// HMI
    pub hmi: Hmi,
    /// [`maneuverRequest_t`] id
    pub id: Option<u16>,
    /// virtual Lines
    virtual_lines: Option<VirtualLines>,
}

impl Lka {
    /// Create a new [Lka]
    pub const fn new() -> Self {
        Self {
            state: StateMachine::new(),
            lkw_state: StateMachine::new(),
            lane: Lane::new(),
            lkw_lane: Lane::new(),
            inhibition: CommonInhibition::new(),
            hmi: Hmi::new(true, true, true),
            id: None,
            virtual_lines: None,
        }
    }
    /// Process and update LKA state and LKW state
    ///
    /// **Note**: this must be the last function called after updating inputs
    pub fn process(&mut self, calib: &Calibration) -> &Self {
        self.state
            .step_lka(calib, &self.inhibition, &self.hmi, &self.lane);
        self.lkw_state.step_lkw(
            calib,
            &self.inhibition,
            &self.hmi,
            &self.state.state,
            &self.lkw_lane,
        );

        if self.state.changed {
            // LKA
            match &self.state.state {
                State::On((OnState::Operational(OperationalState::Active), _)) => {
                    self.virtual_lines = self.get_virtual_lines(LineSide::Left, calib);
                }
                State::On((_, OnState::Operational(OperationalState::Active))) => {
                    self.virtual_lines = self.get_virtual_lines(LineSide::Right, calib);
                }
                _ => {}
            }
        }
        self
    }

    /// Update ego inputs
    #[inline(always)]
    pub fn update_inhibition(&mut self, ego: &Ego, calib: &Calibration) -> &mut Self {
        self.inhibition.update(ego, calib);
        self
    }
    /// Update lane
    #[inline(always)]
    pub fn update_lane(&mut self, lane: Lane) -> &mut Self {
        self.lkw_lane = lane.clone();
        self.lane = lane;
        self
    }

    /// Left [`Line`] is detected
    pub fn is_left_detected(&self) -> bool {
        self.lane.left.is_some()
    }

    /// Right [`Line`] is detected
    pub fn is_right_detected(&self) -> bool {
        self.lane.right.is_some()
    }

    /// Get Virtual Lines
    pub fn get_virtual_lines(
        &mut self,
        side: LineSide,
        calib: &Calibration,
    ) -> Option<VirtualLines> {
        let width = self.lane.width_from_calibration(calib);
        let (line, is_high_hyst) = match side {
            LineSide::Left => (
                self.lane.left.as_ref()?,
                self.state.hyst_state.re_nextline_left,
            ),
            LineSide::Right => (
                self.lane.left.as_ref()?,
                self.state.hyst_state.re_nextline_right,
            ),
        };

        let (re_nextline_too_close, offset) =
            self.lane
                .is_re_nextlane_situation(calib, side, is_high_hyst);

        line.compute_virtual_lines(
            calib,
            self.hmi.sensibility,
            self.inhibition.lat_speed.value.value,
            width,
            offset,
            re_nextline_too_close && !self.hmi.lka_dl_activated,
        )
    }

    /// Get re bextline situation
    pub fn get_re_nextline_status(&mut self, side: LineSide, calib: &Calibration) -> (bool, f32) {
        let is_high_hyst: bool = match side {
            LineSide::Left => self.state.hyst_state.re_nextline_left,
            LineSide::Right => self.state.hyst_state.re_nextline_right,
        };
        let (re_situation_too_close, offset) =
            self.lane
                .is_re_nextlane_situation(calib, side, is_high_hyst);
        (re_situation_too_close, offset)
    }
    /// Get Crossed status
    pub fn get_crossed_status(&mut self, side: LineSide, calib: &Calibration) -> Option<Crossed> {
        let (re_situation_too_close, offset) = self.get_re_nextline_status(side, calib);
        self.lane.virtual_line_crossed_status(
            calib,
            side,
            self.hmi.sensibility,
            self.inhibition.lat_speed.value.value,
            offset,
            !self.hmi.lka_dl_activated && re_situation_too_close,
        )
    }

    /// Get LKW Crossed status (similar as LDW)
    pub fn get_lkw_crossed_status(&self, side: LineSide, calib: &Calibration) -> Option<Crossed> {
        self.lkw_lane.lkw_virtual_line_crossed_status(
            calib,
            side,
            self.hmi.sensibility,
            self.inhibition.lat_speed.value.value,
        )
    }
}

impl From<&Lka> for LKADisplay_t {
    fn from(val: &Lka) -> Self {
        match &val.state.state {
            State::Failure(_) => unimplemented!("LKA Failure management not yet implemented !!"),
            State::Off => LKADisplay_t {
                AppID: ApplicationID_t::LKA,
                Alert: PositionAlert_t::PA_NoAlert,
                OvertimeAlert: TimeAlert_t::TA_NoAlert,
                TakeOverAlert: TimeAlert_t::TA_NoAlert,
            },
            State::On(state) => {
                let alert = if let OnState::Operational(OperationalState::Active) = state.0 {
                    PositionAlert_t::PA_LeftAlert
                } else if let OnState::Operational(OperationalState::Active) = state.1 {
                    PositionAlert_t::PA_RightAlert
                } else {
                    PositionAlert_t::PA_NoAlert
                };
                LKADisplay_t {
                    AppID: ApplicationID_t::LKA,
                    Alert: alert,
                    OvertimeAlert: TimeAlert_t::TA_NoAlert, // HOD
                    TakeOverAlert: TimeAlert_t::TA_NoAlert,
                }
            }
        }
    }
}

impl From<&Lka> for LKAState_t {
    fn from(val: &Lka) -> Self {
        match &val.state.state {
            State::Failure(_) => unimplemented!("LKA Failure management not yet implemented !!"),
            State::Off => LKAState_t {
                AppID: ApplicationID_t::LKA,
                State: ApplicationState_t::AS_OFF,
                Type: FailureType_t::FT_NOFAILURE,
            },
            State::On(state) => match state {
                (OnState::StandBy, OnState::StandBy) => LKAState_t {
                    AppID: ApplicationID_t::LKA,
                    State: ApplicationState_t::AS_RESUME,
                    Type: FailureType_t::FT_NOFAILURE,
                },
                _ => LKAState_t {
                    AppID: ApplicationID_t::LKA,
                    State: ApplicationState_t::AS_OPERATIONAL,
                    Type: FailureType_t::FT_NOFAILURE,
                },
            },
        }
    }
}

/// Compute AVOID/FOLLOW [`maneuverSettings_t`]
fn compute_maneuver_settings(
    line: &Line,
    heading_line_mini: f32,
    no_near_line: f32,
) -> [maneuverSettings_t; 2] {
    let ifid: ifid_t = ifid_t::IID_ROADOBJECTS;
    let ifid_id: u32 = line.id;
    let objective_type: objectiveType_t = objectiveType_t::OBJTYPE_AVOID;
    let ego_ref_point: linecrossingrefpoints_t = match &line.side {
        LineSide::Left => linecrossingrefpoints_t::LCRP_FRONTLEFT,
        LineSide::Right => linecrossingrefpoints_t::LCRP_FRONTRIGHT,
    };
    let avoid = maneuverSettings_t {
        timestampReference: 0,
        ifid,
        ifidID: ifid_id,
        objectiveType: objective_type,
        hardDistLongi: 0.0,
        hardDistLat: 0.0,
        softDistLongi: 0.0,
        softDistLat: 0.0,
        egoRefPoint: ego_ref_point,
    };
    let mut follow = avoid.clone();
    follow.objectiveType = objectiveType_t::OBJTYPE_FOLLOW;
    follow.hardDistLat = (heading_line_mini + no_near_line) / 2.0; // changed to no_near_extreme
    follow.softDistLat = (heading_line_mini - no_near_line).abs() / 2.0;
    [avoid, follow]
}

/// compute rajectory Kinematics Limits
/// TODO: check what must be filled here
fn compute_taj_kin_limits() -> trajKinLimits_t {
    // FIXME: change this to correct values
    use adas_db::calibrations::*;
    trajKinLimits_t {
        jerkLongiMinMax: [-20.0, 20.0],
        jerkLatMinMax: [-20.0, 20.0],
        accelLongiMinMax: [P_tng_mps2_LS_AccNegMax_LSS, P_tng_mps2_LS_AccPosMax_LSS],
        accelLatMinMax: [
            -P_tng_mps2_LS_LatAccMaxLC_LSS,
            P_tng_mps2_LS_LatAccMaxLC_LSS,
        ],
        speedMinMax: [
            P_tng_kmh_LS_MinSpeedDeact_LSS / 3.6,
            P_tng_kmh_LS_MaxSpeedDeact_LSS / 3.6,
        ],
        swaMinMax: [-6.0, 6.0],
        swaSpeedMinMax: [-100.0, 100.0],
    }
}

impl From<&Lka> for Option<maneuverRequest_t> {
    fn from(val: &Lka) -> Self {
        match &val.state.state {
            State::Failure(_) => None,
            State::Off => None,
            State::On(state) => match state {
                (OnState::Operational(OperationalState::Active), _) => {
                    if let (Some(line), Some(v_lines)) =
                        (val.lane.left.as_ref(), val.virtual_lines.as_ref())
                    {
                        Some(maneuverRequest_t {
                            drivingMode: drivingMode_t::DRVMODE_SAFETY,
                            controlAxis: maneuverControlAxis_t::MANCA_LAT,
                            nbifid: 1,
                            maneuverSettings: compute_maneuver_settings(
                                line,
                                v_lines.heading_line_mini,
                                v_lines.no_near_line,
                            ),
                            followTime: 0.0,
                            hardKinLimits: compute_taj_kin_limits(),
                            setSpeed: 0.0,
                            maneuverTimeWindow: 10.0,
                        })
                    } else {
                        error!("Line/Virtual Lines is None, cannot compute maneuvreRequest");
                        None
                    }
                }
                (_, OnState::Operational(OperationalState::Active)) => {
                    if let (Some(line), Some(v_lines)) =
                        (val.lane.right.as_ref(), val.virtual_lines.as_ref())
                    {
                        Some(maneuverRequest_t {
                            drivingMode: drivingMode_t::DRVMODE_SAFETY,
                            controlAxis: maneuverControlAxis_t::MANCA_LAT,
                            nbifid: 1,
                            maneuverSettings: compute_maneuver_settings(
                                line,
                                v_lines.heading_line_mini,
                                v_lines.no_near_line,
                            ),
                            followTime: 0.0,
                            hardKinLimits: compute_taj_kin_limits(),
                            setSpeed: 0.0,
                            maneuverTimeWindow: 10.0,
                        })
                    } else {
                        error!("Line/Virtual Lines is None, cannot compute maneuvreRequest");
                        None
                    }
                }
                _ => None,
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_lka_new() {
        let mut lka = Lka::new();
        let calib = Calibration::new();
        lka.update_inhibition(&Ego::new(), &calib)
            .update_lane(Lane::new());
        lka.process(&calib);
    }
}
