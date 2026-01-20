//! Settings plugin - Graphics, Audio, Controls, and MORE!

use bevy::prelude::*;
use bevy::window::{PresentMode, WindowMode, WindowResolution};

pub struct SettingsPlugin;

impl Plugin for SettingsPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<GameSettings>()
            .init_resource::<SettingsMenuState>()
            .add_systems(Update, (toggle_settings_menu, update_settings, apply_settings));
    }
}

/// All game settings
#[derive(Resource)]
pub struct GameSettings {
    // Graphics
    pub vsync: bool,
    pub fullscreen: bool,
    pub resolution: Resolution,
    pub render_scale: f32,
    pub shadows_enabled: bool,
    pub shadow_quality: ShadowQuality,
    pub ambient_occlusion: bool,
    pub bloom: bool,
    pub bloom_intensity: f32,
    pub anti_aliasing: AntiAliasing,
    pub fov: f32,
    pub max_fps: MaxFps,

    // Visual Effects
    pub fog_enabled: bool,
    pub fog_density: f32,
    pub vignette_enabled: bool,
    pub vignette_intensity: f32,
    pub screen_shake_intensity: f32,
    pub color_theme: ColorTheme,
    pub trail_enabled: bool,
    pub zone_opacity: f32,

    // Display
    pub show_fps: bool,
    pub show_stats: bool,
    pub show_debug_info: bool,
    pub crosshair_style: CrosshairStyle,
    pub crosshair_color: Color,
    pub crosshair_size: f32,
    pub ui_scale: f32,

    // Physics
    pub physics_substeps: u32,
    pub max_bodies: u32,
    pub sleep_threshold: f32,
    pub gravity_strength: f32,

    // Controls
    pub mouse_sensitivity: f32,
    pub invert_y: bool,
    pub sprint_toggle: bool,
    pub camera_smoothing: f32,

    // Audio
    pub master_volume: f32,
    pub sfx_volume: f32,
    pub music_volume: f32,
    pub mute_when_unfocused: bool,

    // Gameplay
    pub auto_save: bool,
    pub spawn_distance: f32,
    pub grab_distance: f32,
    pub explosion_screen_shake: bool,
    pub particle_density: ParticleDensity,

    // Accessibility
    pub colorblind_mode: ColorblindMode,
    pub high_contrast: bool,
    pub reduce_motion: bool,
    pub larger_text: bool,
}

impl Default for GameSettings {
    fn default() -> Self {
        Self {
            // Graphics
            vsync: true,
            fullscreen: false,
            resolution: Resolution::R1920x1080,
            render_scale: 1.0,
            shadows_enabled: true,
            shadow_quality: ShadowQuality::High,
            ambient_occlusion: true,
            bloom: true,
            bloom_intensity: 0.15,
            anti_aliasing: AntiAliasing::Msaa4x,
            fov: 75.0,
            max_fps: MaxFps::Fps120,

            // Visual Effects
            fog_enabled: true,
            fog_density: 1.0,
            vignette_enabled: true,
            vignette_intensity: 0.3,
            screen_shake_intensity: 1.0,
            color_theme: ColorTheme::Pastel,
            trail_enabled: true,
            zone_opacity: 1.0,

            // Display
            show_fps: true,
            show_stats: true,
            show_debug_info: false,
            crosshair_style: CrosshairStyle::Cross,
            crosshair_color: Color::WHITE,
            crosshair_size: 1.0,
            ui_scale: 1.0,

            // Physics
            physics_substeps: 2,
            max_bodies: 1000,
            sleep_threshold: 0.1,
            gravity_strength: 9.81,

            // Controls
            mouse_sensitivity: 1.0,
            invert_y: false,
            sprint_toggle: false,
            camera_smoothing: 0.0,

            // Audio
            master_volume: 1.0,
            sfx_volume: 1.0,
            music_volume: 0.5,
            mute_when_unfocused: true,

            // Gameplay
            auto_save: false,
            spawn_distance: 3.0,
            grab_distance: 50.0,
            explosion_screen_shake: true,
            particle_density: ParticleDensity::High,

            // Accessibility
            colorblind_mode: ColorblindMode::None,
            high_contrast: false,
            reduce_motion: false,
            larger_text: false,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Resolution {
    R800x600,
    R1024x768,
    R1280x720,
    R1366x768,
    R1600x900,
    #[default]
    R1920x1080,
    R2560x1440,
    R3840x2160,
}

impl Resolution {
    pub fn dimensions(&self) -> (f32, f32) {
        match self {
            Self::R800x600 => (800.0, 600.0),
            Self::R1024x768 => (1024.0, 768.0),
            Self::R1280x720 => (1280.0, 720.0),
            Self::R1366x768 => (1366.0, 768.0),
            Self::R1600x900 => (1600.0, 900.0),
            Self::R1920x1080 => (1920.0, 1080.0),
            Self::R2560x1440 => (2560.0, 1440.0),
            Self::R3840x2160 => (3840.0, 2160.0),
        }
    }

    pub fn name(&self) -> &'static str {
        match self {
            Self::R800x600 => "800x600",
            Self::R1024x768 => "1024x768",
            Self::R1280x720 => "1280x720 (720p)",
            Self::R1366x768 => "1366x768",
            Self::R1600x900 => "1600x900",
            Self::R1920x1080 => "1920x1080 (1080p)",
            Self::R2560x1440 => "2560x1440 (1440p)",
            Self::R3840x2160 => "3840x2160 (4K)",
        }
    }

    pub fn all() -> &'static [Self] {
        &[
            Self::R800x600,
            Self::R1024x768,
            Self::R1280x720,
            Self::R1366x768,
            Self::R1600x900,
            Self::R1920x1080,
            Self::R2560x1440,
            Self::R3840x2160,
        ]
    }

    pub fn next(&self) -> Self {
        match self {
            Self::R800x600 => Self::R1024x768,
            Self::R1024x768 => Self::R1280x720,
            Self::R1280x720 => Self::R1366x768,
            Self::R1366x768 => Self::R1600x900,
            Self::R1600x900 => Self::R1920x1080,
            Self::R1920x1080 => Self::R2560x1440,
            Self::R2560x1440 => Self::R3840x2160,
            Self::R3840x2160 => Self::R800x600,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ShadowQuality {
    Off,
    Low,
    Medium,
    #[default]
    High,
    Ultra,
}

impl ShadowQuality {
    pub fn name(&self) -> &'static str {
        match self {
            Self::Off => "Off",
            Self::Low => "Low",
            Self::Medium => "Medium",
            Self::High => "High",
            Self::Ultra => "Ultra",
        }
    }

    pub fn next(&self) -> Self {
        match self {
            Self::Off => Self::Low,
            Self::Low => Self::Medium,
            Self::Medium => Self::High,
            Self::High => Self::Ultra,
            Self::Ultra => Self::Off,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum AntiAliasing {
    Off,
    Fxaa,
    #[default]
    Msaa4x,
    Msaa8x,
}

impl AntiAliasing {
    pub fn name(&self) -> &'static str {
        match self {
            Self::Off => "Off",
            Self::Fxaa => "FXAA",
            Self::Msaa4x => "MSAA 4x",
            Self::Msaa8x => "MSAA 8x",
        }
    }

    pub fn next(&self) -> Self {
        match self {
            Self::Off => Self::Fxaa,
            Self::Fxaa => Self::Msaa4x,
            Self::Msaa4x => Self::Msaa8x,
            Self::Msaa8x => Self::Off,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum MaxFps {
    Fps30,
    Fps60,
    #[default]
    Fps120,
    Fps144,
    Fps240,
    Unlimited,
}

impl MaxFps {
    pub fn value(&self) -> Option<f64> {
        match self {
            Self::Fps30 => Some(30.0),
            Self::Fps60 => Some(60.0),
            Self::Fps120 => Some(120.0),
            Self::Fps144 => Some(144.0),
            Self::Fps240 => Some(240.0),
            Self::Unlimited => None,
        }
    }

    pub fn name(&self) -> &'static str {
        match self {
            Self::Fps30 => "30 FPS",
            Self::Fps60 => "60 FPS",
            Self::Fps120 => "120 FPS",
            Self::Fps144 => "144 FPS",
            Self::Fps240 => "240 FPS",
            Self::Unlimited => "Unlimited",
        }
    }

    pub fn next(&self) -> Self {
        match self {
            Self::Fps30 => Self::Fps60,
            Self::Fps60 => Self::Fps120,
            Self::Fps120 => Self::Fps144,
            Self::Fps144 => Self::Fps240,
            Self::Fps240 => Self::Unlimited,
            Self::Unlimited => Self::Fps30,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum CrosshairStyle {
    None,
    Dot,
    #[default]
    Cross,
    Circle,
    Square,
}

impl CrosshairStyle {
    pub fn name(&self) -> &'static str {
        match self {
            Self::None => "None",
            Self::Dot => "Dot",
            Self::Cross => "Cross",
            Self::Circle => "Circle",
            Self::Square => "Square",
        }
    }

    pub fn next(&self) -> Self {
        match self {
            Self::None => Self::Dot,
            Self::Dot => Self::Cross,
            Self::Cross => Self::Circle,
            Self::Circle => Self::Square,
            Self::Square => Self::None,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ParticleDensity {
    Off,
    Low,
    Medium,
    #[default]
    High,
    Ultra,
}

impl ParticleDensity {
    pub fn name(&self) -> &'static str {
        match self {
            Self::Off => "Off",
            Self::Low => "Low",
            Self::Medium => "Medium",
            Self::High => "High",
            Self::Ultra => "Ultra",
        }
    }

    pub fn multiplier(&self) -> f32 {
        match self {
            Self::Off => 0.0,
            Self::Low => 0.25,
            Self::Medium => 0.5,
            Self::High => 1.0,
            Self::Ultra => 2.0,
        }
    }

    pub fn next(&self) -> Self {
        match self {
            Self::Off => Self::Low,
            Self::Low => Self::Medium,
            Self::Medium => Self::High,
            Self::High => Self::Ultra,
            Self::Ultra => Self::Off,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ColorblindMode {
    #[default]
    None,
    Protanopia,
    Deuteranopia,
    Tritanopia,
}

impl ColorblindMode {
    pub fn name(&self) -> &'static str {
        match self {
            Self::None => "None",
            Self::Protanopia => "Protanopia (Red-Blind)",
            Self::Deuteranopia => "Deuteranopia (Green-Blind)",
            Self::Tritanopia => "Tritanopia (Blue-Blind)",
        }
    }

    pub fn next(&self) -> Self {
        match self {
            Self::None => Self::Protanopia,
            Self::Protanopia => Self::Deuteranopia,
            Self::Deuteranopia => Self::Tritanopia,
            Self::Tritanopia => Self::None,
        }
    }
}

/// Settings menu state
#[derive(Resource, Default)]
pub struct SettingsMenuState {
    pub open: bool,
    pub current_tab: SettingsTab,
    pub pending_changes: bool,
}

/// Color theme for objects
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ColorTheme {
    #[default]
    Pastel,   // Soft, clean colors (coral, mint, sky, etc.)
    Vibrant,  // Bright, saturated colors
    Classic,  // Original colors
    Neon,     // Cyberpunk neon glow
    Monochrome, // Grayscale
}

impl ColorTheme {
    pub fn name(&self) -> &'static str {
        match self {
            Self::Pastel => "Pastel (Clean)",
            Self::Vibrant => "Vibrant",
            Self::Classic => "Classic",
            Self::Neon => "Neon",
            Self::Monochrome => "Monochrome",
        }
    }

    pub fn next(&self) -> Self {
        match self {
            Self::Pastel => Self::Vibrant,
            Self::Vibrant => Self::Classic,
            Self::Classic => Self::Neon,
            Self::Neon => Self::Monochrome,
            Self::Monochrome => Self::Pastel,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum SettingsTab {
    #[default]
    Graphics,
    Visual,
    Display,
    Physics,
    Controls,
    Audio,
    Gameplay,
    Accessibility,
}

impl SettingsTab {
    pub fn name(&self) -> &'static str {
        match self {
            Self::Graphics => "Graphics",
            Self::Visual => "Visual FX",
            Self::Display => "Display",
            Self::Physics => "Physics",
            Self::Controls => "Controls",
            Self::Audio => "Audio",
            Self::Gameplay => "Gameplay",
            Self::Accessibility => "Accessibility",
        }
    }

    pub fn all() -> &'static [Self] {
        &[
            Self::Graphics,
            Self::Visual,
            Self::Display,
            Self::Physics,
            Self::Controls,
            Self::Audio,
            Self::Gameplay,
            Self::Accessibility,
        ]
    }

    pub fn next(&self) -> Self {
        match self {
            Self::Graphics => Self::Visual,
            Self::Visual => Self::Display,
            Self::Display => Self::Physics,
            Self::Physics => Self::Controls,
            Self::Controls => Self::Audio,
            Self::Audio => Self::Gameplay,
            Self::Gameplay => Self::Accessibility,
            Self::Accessibility => Self::Graphics,
        }
    }

    pub fn prev(&self) -> Self {
        match self {
            Self::Graphics => Self::Accessibility,
            Self::Visual => Self::Graphics,
            Self::Display => Self::Visual,
            Self::Physics => Self::Display,
            Self::Controls => Self::Physics,
            Self::Audio => Self::Controls,
            Self::Gameplay => Self::Audio,
            Self::Accessibility => Self::Gameplay,
        }
    }
}

fn toggle_settings_menu(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut menu_state: ResMut<SettingsMenuState>,
) {
    if keyboard.just_pressed(KeyCode::F10) {
        menu_state.open = !menu_state.open;
    }

    if menu_state.open {
        // Tab navigation
        if keyboard.just_pressed(KeyCode::Tab) {
            if keyboard.pressed(KeyCode::ShiftLeft) {
                menu_state.current_tab = menu_state.current_tab.prev();
            } else {
                menu_state.current_tab = menu_state.current_tab.next();
            }
        }
    }
}

fn update_settings(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut settings: ResMut<GameSettings>,
    menu_state: Res<SettingsMenuState>,
) {
    // Quick toggles that work even without menu open
    if keyboard.just_pressed(KeyCode::F11) {
        settings.fullscreen = !settings.fullscreen;
    }

    if keyboard.just_pressed(KeyCode::F3) {
        settings.show_fps = !settings.show_fps;
    }

    // Settings adjustments when menu is open
    if menu_state.open {
        match menu_state.current_tab {
            SettingsTab::Graphics => {
                if keyboard.just_pressed(KeyCode::KeyV) {
                    settings.vsync = !settings.vsync;
                }
                if keyboard.just_pressed(KeyCode::KeyR) {
                    settings.resolution = settings.resolution.next();
                }
                if keyboard.just_pressed(KeyCode::KeyS) {
                    settings.shadow_quality = settings.shadow_quality.next();
                }
                if keyboard.just_pressed(KeyCode::KeyA) {
                    settings.anti_aliasing = settings.anti_aliasing.next();
                }
                if keyboard.just_pressed(KeyCode::KeyB) {
                    settings.bloom = !settings.bloom;
                }
            }
            SettingsTab::Visual => {
                // [B] Bloom toggle, [Up/Down] Bloom intensity
                if keyboard.just_pressed(KeyCode::KeyB) {
                    settings.bloom = !settings.bloom;
                }
                if keyboard.just_pressed(KeyCode::ArrowUp) {
                    settings.bloom_intensity = (settings.bloom_intensity + 0.05).min(1.0);
                }
                if keyboard.just_pressed(KeyCode::ArrowDown) {
                    settings.bloom_intensity = (settings.bloom_intensity - 0.05).max(0.0);
                }
                // [F] Fog toggle, [Shift+Up/Down] Fog density
                if keyboard.just_pressed(KeyCode::KeyF) {
                    settings.fog_enabled = !settings.fog_enabled;
                }
                if keyboard.pressed(KeyCode::ShiftLeft) {
                    if keyboard.just_pressed(KeyCode::ArrowUp) {
                        settings.fog_density = (settings.fog_density + 0.1).min(2.0);
                    }
                    if keyboard.just_pressed(KeyCode::ArrowDown) {
                        settings.fog_density = (settings.fog_density - 0.1).max(0.0);
                    }
                }
                // [G] Vignette toggle
                if keyboard.just_pressed(KeyCode::KeyG) {
                    settings.vignette_enabled = !settings.vignette_enabled;
                }
                // [T] Color theme cycle
                if keyboard.just_pressed(KeyCode::KeyT) {
                    settings.color_theme = settings.color_theme.next();
                }
                // [R] Trails toggle
                if keyboard.just_pressed(KeyCode::KeyR) {
                    settings.trail_enabled = !settings.trail_enabled;
                }
                // [Z] Zone opacity
                if keyboard.just_pressed(KeyCode::KeyZ) {
                    settings.zone_opacity = if settings.zone_opacity > 0.5 { 0.3 } else { 1.0 };
                }
                // [E] Screen shake toggle
                if keyboard.just_pressed(KeyCode::KeyE) {
                    settings.explosion_screen_shake = !settings.explosion_screen_shake;
                }
            }
            SettingsTab::Display => {
                if keyboard.just_pressed(KeyCode::KeyF) {
                    settings.show_fps = !settings.show_fps;
                }
                if keyboard.just_pressed(KeyCode::KeyC) {
                    settings.crosshair_style = settings.crosshair_style.next();
                }
            }
            SettingsTab::Physics => {
                if keyboard.just_pressed(KeyCode::ArrowUp) {
                    settings.gravity_strength = (settings.gravity_strength + 1.0).min(50.0);
                }
                if keyboard.just_pressed(KeyCode::ArrowDown) {
                    settings.gravity_strength = (settings.gravity_strength - 1.0).max(0.0);
                }
            }
            SettingsTab::Controls => {
                if keyboard.just_pressed(KeyCode::ArrowUp) {
                    settings.mouse_sensitivity = (settings.mouse_sensitivity + 0.1).min(3.0);
                }
                if keyboard.just_pressed(KeyCode::ArrowDown) {
                    settings.mouse_sensitivity = (settings.mouse_sensitivity - 0.1).max(0.1);
                }
                if keyboard.just_pressed(KeyCode::KeyI) {
                    settings.invert_y = !settings.invert_y;
                }
            }
            SettingsTab::Audio => {
                if keyboard.just_pressed(KeyCode::ArrowUp) {
                    settings.master_volume = (settings.master_volume + 0.1).min(1.0);
                }
                if keyboard.just_pressed(KeyCode::ArrowDown) {
                    settings.master_volume = (settings.master_volume - 0.1).max(0.0);
                }
                if keyboard.just_pressed(KeyCode::KeyM) {
                    settings.master_volume = if settings.master_volume > 0.0 { 0.0 } else { 1.0 };
                }
            }
            SettingsTab::Gameplay => {
                if keyboard.just_pressed(KeyCode::KeyP) {
                    settings.particle_density = settings.particle_density.next();
                }
                if keyboard.just_pressed(KeyCode::KeyE) {
                    settings.explosion_screen_shake = !settings.explosion_screen_shake;
                }
            }
            SettingsTab::Accessibility => {
                if keyboard.just_pressed(KeyCode::KeyC) {
                    settings.colorblind_mode = settings.colorblind_mode.next();
                }
                if keyboard.just_pressed(KeyCode::KeyH) {
                    settings.high_contrast = !settings.high_contrast;
                }
                if keyboard.just_pressed(KeyCode::KeyL) {
                    settings.larger_text = !settings.larger_text;
                }
            }
        }
    }
}

fn apply_settings(
    settings: Res<GameSettings>,
    mut windows: Query<&mut Window>,
) {
    if !settings.is_changed() {
        return;
    }

    for mut window in windows.iter_mut() {
        // Apply fullscreen
        window.mode = if settings.fullscreen {
            WindowMode::BorderlessFullscreen(MonitorSelection::Current)
        } else {
            WindowMode::Windowed
        };

        // Apply resolution
        let (width, height) = settings.resolution.dimensions();
        window.resolution = WindowResolution::new(width, height);

        // Apply vsync
        window.present_mode = if settings.vsync {
            PresentMode::AutoVsync
        } else {
            PresentMode::AutoNoVsync
        };
    }
}

/// Preset graphics settings
pub struct GraphicsPreset;

impl GraphicsPreset {
    pub fn low(settings: &mut GameSettings) {
        settings.shadows_enabled = false;
        settings.shadow_quality = ShadowQuality::Off;
        settings.ambient_occlusion = false;
        settings.bloom = false;
        settings.anti_aliasing = AntiAliasing::Off;
        settings.render_scale = 0.75;
        settings.particle_density = ParticleDensity::Low;
    }

    pub fn medium(settings: &mut GameSettings) {
        settings.shadows_enabled = true;
        settings.shadow_quality = ShadowQuality::Medium;
        settings.ambient_occlusion = false;
        settings.bloom = true;
        settings.anti_aliasing = AntiAliasing::Fxaa;
        settings.render_scale = 1.0;
        settings.particle_density = ParticleDensity::Medium;
    }

    pub fn high(settings: &mut GameSettings) {
        settings.shadows_enabled = true;
        settings.shadow_quality = ShadowQuality::High;
        settings.ambient_occlusion = true;
        settings.bloom = true;
        settings.anti_aliasing = AntiAliasing::Msaa4x;
        settings.render_scale = 1.0;
        settings.particle_density = ParticleDensity::High;
    }

    pub fn ultra(settings: &mut GameSettings) {
        settings.shadows_enabled = true;
        settings.shadow_quality = ShadowQuality::Ultra;
        settings.ambient_occlusion = true;
        settings.bloom = true;
        settings.anti_aliasing = AntiAliasing::Msaa8x;
        settings.render_scale = 1.0;
        settings.particle_density = ParticleDensity::Ultra;
    }
}
