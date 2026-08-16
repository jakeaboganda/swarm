use bevy::prelude::*;
use viz::EntityId;

// Follow-camera tuning. Constants for now; config-driven tweaking comes after
// the automotive sandbox plan.
//
/// How far behind the tracked entity (along its heading) the camera trails.
const FOLLOW_DISTANCE: f32 = 12.0;
/// How high above the tracked entity the camera rides.
const FOLLOW_HEIGHT: f32 = 6.0;
/// How fast the camera eases toward its target pose (per second).
const FOLLOW_SMOOTH: f32 = 4.0;

/// Which entity the camera is following, or `None` for the overview. A
/// viewer-local choice: nothing about it is streamed.
#[derive(Resource, Default)]
pub struct FollowCam {
    pub target: Option<EntityId>,
}

/// Marks a viewer entity the camera can follow (dynamic agents), carrying its
/// id (for a stable cycle order) and display name.
#[derive(Component)]
pub struct Followable {
    pub id: EntityId,
    pub name: String,
}

/// Marks the on-screen "Following: <name>" readout.
#[derive(Component)]
pub struct FollowLabel;

/// Spawns the (initially blank) follow readout in a corner.
pub fn setup_follow_label(mut commands: Commands) {
    commands.spawn((
        Text::new(""),
        TextColor(Color::WHITE),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(8.0),
            left: Val::Px(8.0),
            ..default()
        },
        FollowLabel,
    ));
}

/// The followable entity ids, in a stable order.
fn sorted_ids(followables: &Query<&Followable>) -> Vec<EntityId> {
    let mut ids: Vec<EntityId> = followables.iter().map(|f| f.id.clone()).collect();
    ids.sort_by(|a, b| a.0.cmp(&b.0));
    ids
}

/// `F` toggles follow on/off; `Tab` cycles to the next followable entity.
pub fn follow_input(
    keys: Res<ButtonInput<KeyCode>>,
    mut follow: ResMut<FollowCam>,
    followables: Query<&Followable>,
) {
    let ids = sorted_ids(&followables);

    if keys.just_pressed(KeyCode::KeyF) {
        follow.target = if follow.target.is_some() {
            None
        } else {
            ids.first().cloned()
        };
    }

    if keys.just_pressed(KeyCode::Tab) && !ids.is_empty() {
        let next = match &follow.target {
            Some(current) => {
                let i = ids
                    .iter()
                    .position(|id| id == current)
                    .map_or(0, |i| (i + 1) % ids.len());
                ids[i].clone()
            }
            None => ids[0].clone(),
        };
        follow.target = Some(next);
    }
}

/// In follow mode, ease the camera to a fixed offset from the tracked entity,
/// looking at it. Reverts to overview if that entity is gone.
pub fn follow_camera(
    time: Res<Time>,
    mut follow: ResMut<FollowCam>,
    targets: Query<(&Followable, &Transform), Without<Camera3d>>,
    mut camera: Query<&mut Transform, With<Camera3d>>,
) {
    let Some(id) = follow.target.clone() else {
        return; // overview: frame_camera handles it
    };
    let Some((_, target)) = targets.iter().find(|(f, _)| f.id == id) else {
        follow.target = None; // followed entity despawned -> overview
        return;
    };
    let Ok(mut camera) = camera.single_mut() else {
        return;
    };
    let focus = target.translation;
    // Trail from behind: project the entity's forward onto the ground plane so
    // the camera sits behind its heading without inheriting pitch/roll (the car
    // pitches on grades and rolls in turns). Fall back to +X if it's degenerate.
    let forward = *target.forward();
    let mut heading = Vec3::new(forward.x, 0.0, forward.z).normalize_or_zero();
    if heading == Vec3::ZERO {
        heading = Vec3::X;
    }
    let desired = focus - heading * FOLLOW_DISTANCE + Vec3::Y * FOLLOW_HEIGHT;
    let alpha = (FOLLOW_SMOOTH * time.delta_secs()).min(1.0);
    camera.translation = camera.translation.lerp(desired, alpha);
    camera.look_at(focus, Vec3::Y);
}

/// Shows "Following: <name>" while following, blank otherwise.
pub fn update_follow_label(
    follow: Res<FollowCam>,
    followables: Query<&Followable>,
    mut label: Query<&mut Text, With<FollowLabel>>,
) {
    let Ok(mut text) = label.single_mut() else {
        return;
    };
    let content = match &follow.target {
        Some(id) => followables
            .iter()
            .find(|f| &f.id == id)
            .map(|f| format!("Following: {}   (Tab: next, F: overview)", f.name))
            .unwrap_or_default(),
        None => String::new(),
    };
    if text.0 != content {
        text.0 = content;
    }
}
