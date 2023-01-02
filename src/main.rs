use std::sync::atomic::AtomicUsize;

use ggez::winit::event::VirtualKeyCode;
use ggez::{event, mint};
use ggez::graphics::{self, Color, DrawParam, Canvas, Drawable, Rect, GraphicsContext, Mesh, Transform, Image};
use ggez::mint::Point2;
use ggez::{Context, GameResult};
use ggez::input::keyboard::KeyInput;
use ggez::glam::*;
use ggez::conf::{WindowMode, WindowSetup};
use rapier2d::na::{OPoint, Translation, Vector2, ComplexField};
use rapier2d::prelude::*;

struct Camera {
    position: Vector<Real>,
    rotation: f32,
    draw_scale: f32,
}

impl Camera {
    fn render(&self, canvas: &mut Canvas, gfx: &GraphicsContext, visibles: Vec<&dyn Visible>) {
        let width = gfx.drawable_size().0;
        let height = gfx.drawable_size().1;
        let position = vector![self.position.x + width / 2.0, self.position.y - height / 2.0];

        for visible in visibles {
            visible.draw(canvas, gfx, &Point2{x: position.x, y: position.y}, self.rotation, height);
        }
    }

    fn update(&mut self, ctx: &Context) {
        const MOVEMENT_SPEED: f32 = 5.0;

        if ctx.keyboard.is_key_pressed(VirtualKeyCode::Up) {
            self.position.y += MOVEMENT_SPEED
        }

        if ctx.keyboard.is_key_pressed(VirtualKeyCode::Down) {
            self.position.y -= MOVEMENT_SPEED
        }

        if ctx.keyboard.is_key_pressed(VirtualKeyCode::Left) {
            self.position.x += MOVEMENT_SPEED
        }

        if ctx.keyboard.is_key_pressed(VirtualKeyCode::Right) {
            self.position.x -= MOVEMENT_SPEED
        }
    }
}

trait Simulated {
    fn get_rigid_body_handle(&self) -> &RigidBodyHandle;
    fn get_collider_handle(&self) -> &ColliderHandle;
}

trait SimulatedVisible: Simulated {
    fn get_mesh(&self) -> Mesh;
    fn get_position(&self) -> &Point2<f32>;
    fn set_position(&mut self, value: Point2<f32>);
    fn get_rotation(&self) -> f32;
    fn set_rotation(&mut self, value: f32);
    fn get_image(&self) -> Option<Image>;
    fn update(&mut self, simulation: &Simulation) {
        self.set_position(Point2{
            x: simulation.collider_set[*self.get_collider_handle()].translation().x,
            y: simulation.collider_set[*self.get_collider_handle()].translation().y
        });

        self.set_rotation(simulation.collider_set[*self.get_collider_handle()].rotation().angle());
    }
}

trait Visible {
    fn draw(&self, canvas: &mut Canvas, gfx: &GraphicsContext, position: &Point2<f32>, rotation: f32, window_height: f32);
}

impl<T: SimulatedVisible> Visible for T {
    fn draw(&self, canvas: &mut Canvas, gfx: &GraphicsContext, position: &Point2<f32>, rotation: f32, window_height: f32) {
            if let Some(image) = self.get_image() {
                canvas.draw_textured_mesh(self.get_mesh(), image, DrawParam::new().dest(Point2{
                    x: self.get_position().x + position.x,
                    y: window_height - self.get_position().y + position.y}).rotation(-self.get_rotation() + rotation))
            }
            else {
                canvas.draw(&self.get_mesh(), DrawParam::new().dest(Point2{
                    x: self.get_position().x + position.x,
                    y: window_height - self.get_position().y + position.y}).rotation(-self.get_rotation() + rotation))
            }
    }
}

fn remap<T>(old_value: T, old_min: T, old_max: T, new_min: T, new_max: T) -> T
where T: std::ops::Add<Output = T> + std::ops::Sub<Output = T> + std::ops::Mul<Output = T> + std::ops::Div<Output = T> + std::marker::Copy {
    let old_range = old_max - old_min;
    let new_range = new_max - new_min;
    let new_value = (((old_value - old_min) * new_range) / old_range) + new_min;

    new_value
}

struct Jump {
    start: Option<f32>,
    end: Option<f32>,
}

impl Jump {
    const BUFFERING: f32 = 0.1;

    fn new() -> Self {
        Jump{start: None, end: None}
    }

    fn start(&mut self, ctx: &Context) {
        self.start = Some(ctx.time.time_since_start().as_secs_f32());
        self.end = None;
    }

    fn end(&mut self, ctx: &Context) {
        self.end = Some(ctx.time.time_since_start().as_secs_f32());
    }

    fn active(&self) -> bool {
        self.start.is_some() && self.end.is_none()
    }

    fn velocity(&self, ctx: &Context) -> f32 {
        const LAUNCH_VELOCITY: f32 = 250.0;
        const TERMINAL_VELOCITY: f32 = -300.0;

        const BEGIN_GRAVITY: f32 = 0.3;
        const REACH_APEX: f32 = 0.6;
        const RELEASE_APEX: f32 = 0.61;
        const REACH_TERMINAL_VELOCITY: f32 = 0.85;

        let last_velocity = if let Some(t) = self.duration(ctx) {
            if t < BEGIN_GRAVITY {
                LAUNCH_VELOCITY
            }
            else if t < REACH_APEX {
                remap(t, BEGIN_GRAVITY, REACH_APEX, LAUNCH_VELOCITY, 0.0)
            }
            else if t < RELEASE_APEX {
                0.0
            }
            else if t < REACH_TERMINAL_VELOCITY {
                remap(t, RELEASE_APEX, REACH_TERMINAL_VELOCITY, 0.0, TERMINAL_VELOCITY)
            }
            else {
                TERMINAL_VELOCITY
            }
        }
        else {
            TERMINAL_VELOCITY
        };

        if let Some(e) = self.end {
            // If we released early, transition to terminal velocity and skip the apex
            let t = ctx.time.time_since_start().as_secs_f32() - e;

            remap(t, 0.0, 0.3, last_velocity, TERMINAL_VELOCITY)
        }
        else {
            last_velocity
        }
    }

    fn duration(&self, ctx: &Context) -> Option<f32> {
        match self.start {
            Some(start) => Some(ctx.time.time_since_start().as_secs_f32() - start),
            None => None,
        }
    }
}

static player_grounded: AtomicUsize = AtomicUsize::new(0);

struct GamePhysicsHooks {
    player_collider_handle: Option<ColliderHandle>,
}

impl PhysicsHooks for GamePhysicsHooks {
    fn modify_solver_contacts(self: &GamePhysicsHooks, context: &mut ContactModificationContext) {
        if let Some(handle) = self.player_collider_handle {
            if context.collider1 == handle && context.normal.x.abs() < 0.75 {
                player_grounded.store(1, core::sync::atomic::Ordering::Relaxed)
            }
        }

        for solver_contact in &mut *context.solver_contacts {
            solver_contact.friction = 0.0;
        }
    }
}

struct Player {
    rigid_body_handle: RigidBodyHandle,
    collider_handle: ColliderHandle,
    mesh: Mesh,
    position: Point2<f32>,
    rotation: f32,
    jump: Jump,
    last_buffered_jump_time: Option<f32>,
    jump_was_pressed: bool,
}

impl Player {
    fn new(simulation: &mut Simulation, gfx: &GraphicsContext, translation: Vector<Real>) -> GameResult<Self> {
        let rigid_body = RigidBodyBuilder::new(RigidBodyType::Dynamic)
            .lock_rotations()
            .translation(translation)
            .can_sleep(false);
        let rigid_body_handle = simulation.rigid_body_set.insert(rigid_body);

        let collider = ColliderBuilder::cuboid(20.0, 30.0)
            .active_hooks(ActiveHooks::MODIFY_SOLVER_CONTACTS)
            .build();
        let collider_handle = simulation.collider_set.insert_with_parent(
            collider.clone(),
            rigid_body_handle,
            &mut simulation.rigid_body_set);

        let half_extents = collider
            .shape()
            .as_cuboid()   
            .unwrap()
            .half_extents;
        let top_left = collider.translation() - half_extents;
        let extents = half_extents * 2.0;

        let mesh = Mesh::new_rectangle(
            gfx,
            graphics::DrawMode::fill(),
            graphics::Rect::new(top_left.x, top_left.y, extents.x, extents.y),
            Color::GREEN)?;

        Ok(Player{
            rigid_body_handle,
            collider_handle,
            mesh,
            position: Point2{x: translation.x, y: translation.y},
            rotation: 0.0,
            jump: Jump::new(),
            last_buffered_jump_time: None,
            jump_was_pressed: false,
        })
    }

    fn movement_update(&mut self, simulation: &mut Simulation, ctx: &Context) {
        const MOVEMENT_FACTOR: f32 = 2.5;

        let mut velocity = simulation.rigid_body_set[*self.get_rigid_body_handle()].linvel().clone();

        if ctx.keyboard.is_key_pressed(VirtualKeyCode::W) {

        }

        if ctx.keyboard.is_key_pressed(VirtualKeyCode::S) {
            
        }

        if ctx.keyboard.is_key_pressed(VirtualKeyCode::A) {
            velocity.x -= MOVEMENT_FACTOR;
        }

        if ctx.keyboard.is_key_pressed(VirtualKeyCode::D) {
            velocity.x += MOVEMENT_FACTOR;
        }

        if ctx.keyboard.is_key_pressed(VirtualKeyCode::Space) {
            if !self.jump_was_pressed {
                self.last_buffered_jump_time = Some(ctx.time.time_since_start().as_secs_f32());
            }

            self.jump_was_pressed = true;
        }
        else {
            self.jump_was_pressed = false;

            if self.jump.active() {
                self.jump.end(ctx);
            }
        }

        let attempting_jump = if let Some(time) = self.last_buffered_jump_time {
            if ctx.time.time_since_start().as_secs_f32() - time < Jump::BUFFERING {
                true
            }
            else {
                false
            }
        }
        else {
            false
        };

        let grounded = player_grounded.load(core::sync::atomic::Ordering::Relaxed);
        let v = self.jump.velocity(ctx);

        if grounded == 1 {
            if attempting_jump && !self.jump.active() {
                self.jump.start(ctx);
                self.last_buffered_jump_time = None;
            }
        }

        if grounded == 0 || self.jump.active() {
            velocity.y = v;
        }

        simulation.rigid_body_set[*self.get_rigid_body_handle()].set_linvel(velocity, true);
    }
}

impl Simulated for Player {
    fn get_rigid_body_handle(&self) -> &RigidBodyHandle {
        &self.rigid_body_handle
    }

    fn get_collider_handle(&self) -> &ColliderHandle {
        &self.collider_handle
    }
}

impl SimulatedVisible for Player {
    fn get_mesh(&self) -> Mesh {
       self.mesh.clone()
    }

    fn get_position(&self) -> &Point2<f32> {
        &self.position
    }

    fn set_position(&mut self, value: Point2<f32>) {
        self.position = value
    }

    fn get_rotation(&self) -> f32 {
        self.rotation
    }

    fn set_rotation(&mut self, value: f32) {
        self.rotation = value
    }

    fn get_image(&self) -> Option<Image> {
        None
    }
}

struct Platform {
    rigid_body_handle: RigidBodyHandle,
    collider_handle: ColliderHandle,
    mesh: Mesh,
    position: Point2<f32>,
    rotation: f32,
}

impl Platform {
    fn from_body_and_collider(simulation: &mut Simulation, gfx: &GraphicsContext, rigid_body: RigidBody, collider: Collider, color: Color) -> GameResult<Self> {
        let translation = rigid_body.translation().clone();
        let rotation = rigid_body.rotation().angle();

        let rigid_body_handle = simulation.rigid_body_set.insert(rigid_body);
        let collider_handle = simulation.collider_set.insert_with_parent(
            collider.clone(),
            rigid_body_handle,
            &mut simulation.rigid_body_set);

        let mesh = match collider.shape().shape_type() {
            ShapeType::Ball => todo!(),
            ShapeType::Cuboid => {
                let half_extents = collider
                    .shape()
                    .as_cuboid()
                    .unwrap()
                .half_extents;
                let top_left = collider.translation() - half_extents;
                let extents = half_extents * 2.0;

                Mesh::new_rectangle(gfx, graphics::DrawMode::fill(), graphics::Rect::new(top_left.x, top_left.y, extents.x, extents.y), color)?
            },
            ShapeType::Capsule => todo!(),
            ShapeType::Segment => todo!(),
            ShapeType::Triangle => todo!(),
            ShapeType::TriMesh => todo!(),
            ShapeType::Polyline => todo!(),
            ShapeType::HalfSpace => todo!(),
            ShapeType::HeightField => todo!(),
            ShapeType::Compound => todo!(),
            ShapeType::ConvexPolygon => todo!(),
            ShapeType::RoundCuboid => todo!(),
            ShapeType::RoundTriangle => todo!(),
            ShapeType::RoundConvexPolygon => todo!(),
            ShapeType::Custom => todo!(),
        };

        Ok(Platform{
            rigid_body_handle,
            collider_handle,
            mesh,
            position: Point2{x: translation.x, y: translation.y},
            rotation: rotation,
        })
    }
}

impl Simulated for Platform {
    fn get_rigid_body_handle(&self) -> &RigidBodyHandle {
        &self.rigid_body_handle
    }

    fn get_collider_handle(&self) -> &ColliderHandle {
        &self.collider_handle
    }
}

impl SimulatedVisible for Platform {
    fn get_mesh(&self) -> Mesh {
        self.mesh.clone()
    }

    fn get_position(&self) -> &Point2<f32> {
        &self.position
    }

    fn set_position(&mut self, value: Point2<f32>) {
        self.position = value
    }

    fn get_rotation(&self) -> f32 {
        self.rotation
    }

    fn set_rotation(&mut self, value: f32) {
        self.rotation = value
    }

    fn get_image(&self) -> Option<Image> {
        None
    }
}

struct Simulation {
    rigid_body_set: RigidBodySet,
    collider_set: ColliderSet,
    gravity: Vector<f32>,
    integration_parameters: IntegrationParameters,
    physics_pipeline: PhysicsPipeline,
    island_manager: IslandManager,
    broad_phase: BroadPhase,
    narrow_phase: NarrowPhase,
    impulse_joint_set: ImpulseJointSet,
    multibody_joint_set: MultibodyJointSet,
    ccd_solver: CCDSolver,
    physics_hooks: GamePhysicsHooks,
    event_handler: ()
}

impl Simulation {
    fn new() -> Self {
        let mut rigid_body_set = RigidBodySet::new();
        let mut collider_set = ColliderSet::new();
        let gravity = vector![0.0, -1000.0];
        let integration_parameters = IntegrationParameters::default();
        let mut physics_pipeline = PhysicsPipeline::new();
        let mut island_manager = IslandManager::new();
        let mut broad_phase = BroadPhase::new();
        let mut narrow_phase = NarrowPhase::new();
        let mut impulse_joint_set = ImpulseJointSet::new();
        let mut multibody_joint_set = MultibodyJointSet::new();
        let mut ccd_solver = CCDSolver::new();
        let physics_hooks = GamePhysicsHooks{player_collider_handle: None};
        let event_handler = ();

        Simulation{
            rigid_body_set, collider_set,
            gravity, integration_parameters, physics_pipeline,
            island_manager, broad_phase, narrow_phase,
            impulse_joint_set, multibody_joint_set, ccd_solver,
            physics_hooks, event_handler
        }
    }

    fn update(&mut self) {
        player_grounded.store(0, core::sync::atomic::Ordering::Relaxed);

        self.physics_pipeline.step(
            &self.gravity,
            &self.integration_parameters,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigid_body_set,
            &mut self.collider_set,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            &mut self.ccd_solver,
            &self.physics_hooks,
            &self.event_handler,
          );
    }
}

struct MainState {
    player: Player,
    simulation: Simulation,
    camera: Camera,
    ground: Platform
}

impl MainState {
    fn new(mut simulation: Simulation, gfx: &GraphicsContext) -> GameResult<MainState> {
        let player = Player::new(&mut simulation, gfx, vector![0.0, 500.0])?;

        simulation.physics_hooks.player_collider_handle = Some(player.collider_handle);

        let ground = Platform::from_body_and_collider(&mut simulation, gfx,
            RigidBodyBuilder::new(RigidBodyType::Dynamic).lock_translations().build(),
            ColliderBuilder::cuboid(500.0, 5.0).density(10.0).build(),
        Color::WHITE)?;

        Ok(MainState{
            player,
            simulation: simulation,
            camera: Camera{position: vector![0.0, 0.0], rotation: 0.0, draw_scale: 1.0},
            ground
        })
    }
}

impl event::EventHandler<ggez::GameError> for MainState {
    fn update(&mut self, ctx: &mut Context) -> GameResult {
        while ctx.time.check_update_time(120) {
            self.player.movement_update(&mut self.simulation, ctx);
            self.simulation.update();
            self.player.update(&mut self.simulation);
            self.ground.update(&mut self.simulation);
            self.camera.update(ctx);
        }

        Ok(())
    }

    fn draw(&mut self, ctx: &mut Context) -> GameResult {
        let mut canvas = graphics::Canvas::from_frame(
            ctx,
            graphics::Color::from([0.1, 0.2, 0.0, 1.0]),
        );

        self.camera.render(&mut canvas, &ctx.gfx, vec![(&self.player) as &dyn Visible, (&self.ground) as &dyn Visible]);

        canvas.finish(ctx)?;
        Ok(())
    }
}

pub fn main() -> GameResult {
    let cb = ggez::ContextBuilder::new("platformer", "techperson")
        .window_setup(ggez::conf::WindowSetup::default().title("Platformer"))
        .window_mode(
            ggez::conf::WindowMode::default()
                .dimensions(1280.0, 720.0)
                .resizable(true),
        );
    
    let (ctx, event_loop) = cb.build()?;
    let mut simulation = Simulation::new();

    let state = MainState::new(simulation, &ctx.gfx)?;
    event::run(ctx, event_loop, state)
}
