use bevy::{
    pbr::{CascadeShadowConfigBuilder, DirectionalLightShadowMap},
    prelude::*,
};
use bevy_rapier3d::prelude::*;

use std::f32::consts::*;

#[derive(Component, Clone, Debug)]
struct DroneMotors {
    left_front: f32,
    right_front: f32,
    left_rear: f32,
    right_rear: f32,
}

fn vec_to_3d(v: Vec4) -> Vec3 {
    Vec3::new(v.x, v.y, v.z)
}

fn calculate_forces(mut drones: Query<(&mut ExternalForce, &DroneMotors, &Transform)>) {
    for (mut force, motors, transform) in &mut drones {
        let trans_mat = transform.compute_matrix();
        let motor_speed_and_pos = [
            (
                motors.left_front,
                vec_to_3d(trans_mat * Vec4::new(1.8, 0.0, 1.8, 0.0)),
            ),
            (
                motors.right_front,
                vec_to_3d(trans_mat * Vec4::new(-1.8, 0.0, 1.8, 0.0)),
            ),
            (
                motors.left_rear,
                vec_to_3d(trans_mat * Vec4::new(1.8, 0.0, -1.8, 0.0)),
            ),
            (
                motors.right_rear,
                vec_to_3d(trans_mat * Vec4::new(-1.8, 0.0, -1.8, 0.0)),
            ),
        ];
        force.force = Vec3::ZERO;
        force.torque = Vec3::ZERO;

        for (motor_speed, motor_pos) in motor_speed_and_pos {
            let motor_force = vec_to_3d(trans_mat * (motor_speed * Vec4::new(0.0, 1.0, 0.0, 0.0)));
            force.torque += motor_pos.cross(motor_force);
            force.force += motor_force;
        }

        force.torque += 0.2
            * vec_to_3d(
                trans_mat
                    * Vec4::new(
                        0.0,
                        motors.left_front + motors.right_rear
                            - motors.right_front
                            - motors.left_rear,
                        0.0,
                        0.0,
                    ),
            );
    }
}

fn main() {
    App::new()
        .insert_resource(DirectionalLightShadowMap { size: 4096 })
        .add_plugins(DefaultPlugins)
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugins(RapierDebugRenderPlugin::default())
        .add_systems(Startup, setup_graphics)
        .add_systems(Startup, setup_physics)
        .add_systems(Update, animate_light_direction)
        .add_systems(Update, calculate_forces)
        .run();
}

fn setup_physics(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    asset_server: Res<AssetServer>,
) {
    // Spawn ground plane entity
    commands
        .spawn(RigidBody::Fixed)
        .insert(Collider::cuboid(100.0, 0.1, 100.0))
        .insert(PbrBundle {
            mesh: meshes.add(Cuboid::new(100.0, 0.1, 100.0)),
            material: materials.add(Color::srgb(0.5, 0.5647, 1.0)),
            ..default()
        })
        .insert(TransformBundle::from(Transform::from_xyz(0.0, 0.0, 0.0)));

    let my_mesh = asset_server.load("uploads_files_4453673_FPV+DRONE.gltf#Scene0");

    // Spawn drone entity
    commands
        .spawn(RigidBody::Dynamic)
        .insert(Collider::cuboid(3.6, 0.8, 3.6))
        .insert(SceneBundle {
            scene: my_mesh,
            ..default()
        })
        .insert(ColliderMassProperties::Mass(0.5))
        .insert(TransformBundle::from(Transform {
            translation: Vec3::new(0.0, 0.6, 0.0),
            scale: Vec3::new(0.06, 0.06, 0.06),
            ..Default::default()
        }))
        .insert(ExternalForce {
            force: Vec3::new(0.0, 0.0, 0.0),
            torque: Vec3::new(0.0, 0.0, 0.0),
        })
        .insert(DroneMotors {
            left_front: 10.0,
            right_front: 15.0,
            left_rear: 15.0,
            right_rear: 10.0,
        });
}

fn setup_graphics(mut commands: Commands) {
    commands.spawn(Camera3dBundle {
        transform: Transform::from_xyz(0.7, 0.7, 1.0).looking_at(Vec3::new(0.0, 0.3, 0.0), Vec3::Y),
        ..default()
    });

    commands.spawn(DirectionalLightBundle {
        directional_light: DirectionalLight {
            shadows_enabled: true,
            ..default()
        },
        cascade_shadow_config: CascadeShadowConfigBuilder {
            num_cascades: 1,
            maximum_distance: 1.6,
            ..default()
        }
        .into(),
        ..default()
    });
}

fn animate_light_direction(
    time: Res<Time>,
    mut query: Query<&mut Transform, With<DirectionalLight>>,
) {
    for mut transform in &mut query {
        transform.rotation = Quat::from_euler(
            EulerRot::ZYX,
            0.0,
            time.elapsed_seconds() * PI / 5.0,
            -FRAC_PI_4,
        );
    }
}
