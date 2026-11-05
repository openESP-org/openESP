//! Aircraft sound mixing, independent of the simulation and rendering layers.
pub mod engine;
use anyhow::{Context, Result, ensure};
use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use esp_assets::sound::Sample;
use esp_formats::cfg::sound::Parameters;
use std::sync::{
    Arc,
    mpsc::{self, Receiver, SyncSender},
};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct VoiceId(pub u64);

pub enum Command {
    Add {
        id: VoiceId,
        sample: Arc<Sample>,
        parameters: Box<Parameters>,
        looping: bool,
    },
    Set {
        id: VoiceId,
        level: f64,
        inside: bool,
        active: bool,
    },
    Remove(VoiceId),
}

enum Message {
    Single(Command),
    Controls(Vec<Command>),
}

fn send_controls(sender: &SyncSender<Message>, commands: Vec<Command>) -> Result<()> {
    for command in &commands {
        ensure!(
            matches!(command, Command::Set { .. }),
            "audio control batch contains a non-control command"
        );
        validate_command(command)?;
    }
    sender
        .try_send(Message::Controls(commands))
        .map_err(|_| anyhow::anyhow!("audio command queue unavailable or full"))
}

struct Voice {
    id: VoiceId,
    sample: Arc<Sample>,
    parameters: Parameters,
    looping: bool,
    position: f64,
    level: f64,
    inside: bool,
    active: bool,
}

/// A deterministic mixer; callers can render to a device or an offline buffer.
pub struct Mixer {
    sample_rate: u32,
    channels: usize,
    voices: Vec<Voice>,
}

fn envelope(points: &[[f64; 2]], x: f64, default: f64) -> f64 {
    let Some(first) = points.first() else {
        return default;
    };
    if x <= first[0] {
        return first[1];
    }
    for pair in points.windows(2) {
        if x <= pair[1][0] {
            let span = pair[1][0] - pair[0][0];
            if span <= 0.0 {
                return pair[1][1];
            }
            return pair[0][1] + (pair[1][1] - pair[0][1]) * (x - pair[0][0]) / span;
        }
    }
    points.last().map_or(default, |point| point[1])
}

// Sample validation can scan millions of frames. Keep it off the device callback.
fn validate_command(command: &Command) -> Result<()> {
    match command {
        Command::Add {
            sample, parameters, ..
        } => {
            ensure!(
                sample.sample_rate > 0
                    && (1..=2).contains(&sample.channels)
                    && !sample.samples.is_empty()
                    && sample
                        .samples
                        .len()
                        .is_multiple_of(sample.channels as usize)
                    && sample.samples.iter().all(|value| value.is_finite()),
                "invalid audio sample"
            );
            for points in [
                parameters.pitch_envelope.as_deref(),
                parameters.volume_envelope.as_deref(),
            ]
            .into_iter()
            .flatten()
            {
                ensure!(
                    points.iter().all(|p| p[0].is_finite() && p[1].is_finite())
                        && points.windows(2).all(|p| p[0][0] <= p[1][0]),
                    "invalid sound envelope"
                );
            }
        }
        Command::Set { level, .. } => ensure!(level.is_finite(), "invalid sound control"),
        Command::Remove(_) => {}
    }
    Ok(())
}

impl Mixer {
    pub fn new(sample_rate: u32, channels: usize) -> Result<Self> {
        ensure!(
            sample_rate > 0 && (1..=32).contains(&channels),
            "invalid audio output format"
        );
        Ok(Self {
            sample_rate,
            channels,
            voices: Vec::new(),
        })
    }

    pub fn command(&mut self, command: Command) -> Result<()> {
        validate_command(&command)?;
        self.apply(command)
    }

    fn apply(&mut self, command: Command) -> Result<()> {
        match command {
            Command::Add {
                id,
                sample,
                parameters,
                looping,
            } => {
                ensure!(
                    !self.voices.iter().any(|voice| voice.id == id),
                    "duplicate sound voice"
                );
                self.voices.push(Voice {
                    id,
                    sample,
                    parameters: *parameters,
                    looping,
                    position: 0.0,
                    level: 0.0,
                    inside: true,
                    active: false,
                });
            }
            Command::Set {
                id,
                level,
                inside,
                active,
            } => {
                if let Some(voice) = self.voices.iter_mut().find(|voice| voice.id == id) {
                    if active && !voice.active {
                        voice.position = 0.0;
                    }
                    voice.level = level.clamp(0.0, 1.0);
                    voice.inside = inside;
                    voice.active = active;
                }
            }
            Command::Remove(id) => self.voices.retain(|voice| voice.id != id),
        }
        Ok(())
    }

    pub fn render(&mut self, output: &mut [f32]) {
        output.fill(0.0);
        for voice in &mut self.voices {
            if !voice.active {
                continue;
            }
            let audible = match voice.parameters.viewpoint {
                Some(1) => voice.inside,
                Some(2) => !voice.inside,
                _ => true,
            };
            let volume = voice
                .parameters
                .volume_envelope
                .as_deref()
                .map_or(100.0, |p| envelope(p, voice.level, 100.0));
            // Authored volume 50 represents -3 dB, so convert power percentage to amplitude.
            let gain = if audible {
                (volume.clamp(0.0, 100.0) / 100.0).sqrt() as f32
            } else {
                0.0
            };
            let pitch = voice
                .parameters
                .pitch_envelope
                .as_deref()
                .map_or(1.0, |p| envelope(p, voice.level, 1.0))
                .clamp(0.01, 8.0);
            let increment =
                pitch * f64::from(voice.sample.sample_rate) / f64::from(self.sample_rate);
            let source_channels = voice.sample.channels as usize;
            let frames = voice.sample.samples.len() / source_channels;
            for frame in output.chunks_exact_mut(self.channels) {
                let index = voice.position as usize;
                if index >= frames {
                    voice.active = false;
                    break;
                }
                let next = if index + 1 < frames {
                    index + 1
                } else if voice.looping {
                    0
                } else {
                    index
                };
                let frac = (voice.position - index as f64) as f32;
                let source = |channel: usize| {
                    let a = voice.sample.samples[index * source_channels + channel];
                    let b = voice.sample.samples[next * source_channels + channel];
                    a + (b - a) * frac
                };
                let left = source(0);
                let right = if source_channels == 2 {
                    source(1)
                } else {
                    left
                };
                if self.channels == 1 {
                    frame[0] += (left + right) * 0.5 * gain;
                } else {
                    frame[0] += left * gain;
                    frame[1] += right * gain;
                }
                voice.position += increment;
                if voice.looping {
                    voice.position %= frames as f64;
                }
            }
        }
        for sample in output {
            *sample = sample.clamp(-1.0, 1.0);
        }
    }
}

/// Owns the audio stream. Commands are delivered without blocking the audio callback.
pub struct Output {
    _stream: cpal::Stream,
    commands: SyncSender<Message>,
    errors: Receiver<String>,
}

impl Output {
    pub fn open_default() -> Result<Self> {
        let host = cpal::default_host();
        let device = host
            .default_output_device()
            .context("no default audio output device")?;
        let config = device.default_output_config()?;
        let channels = usize::from(config.channels());
        let sample_rate = config.sample_rate().0;
        let (commands, receiver) = mpsc::sync_channel::<Message>(256);
        let (error_sender, errors) = mpsc::channel();
        let mixer = Mixer::new(sample_rate, channels)?;
        let build = |mixer, receiver| match config.sample_format() {
            cpal::SampleFormat::F32 => stream::<f32>(
                &device,
                &config.clone().into(),
                mixer,
                receiver,
                error_sender,
            ),
            cpal::SampleFormat::F64 => stream::<f64>(
                &device,
                &config.clone().into(),
                mixer,
                receiver,
                error_sender,
            ),
            cpal::SampleFormat::I8 => stream::<i8>(
                &device,
                &config.clone().into(),
                mixer,
                receiver,
                error_sender,
            ),
            cpal::SampleFormat::I16 => stream::<i16>(
                &device,
                &config.clone().into(),
                mixer,
                receiver,
                error_sender,
            ),
            cpal::SampleFormat::I32 => stream::<i32>(
                &device,
                &config.clone().into(),
                mixer,
                receiver,
                error_sender,
            ),
            cpal::SampleFormat::U8 => stream::<u8>(
                &device,
                &config.clone().into(),
                mixer,
                receiver,
                error_sender,
            ),
            cpal::SampleFormat::U16 => stream::<u16>(
                &device,
                &config.clone().into(),
                mixer,
                receiver,
                error_sender,
            ),
            cpal::SampleFormat::U32 => stream::<u32>(
                &device,
                &config.clone().into(),
                mixer,
                receiver,
                error_sender,
            ),
            other => anyhow::bail!("unsupported default audio sample format {other:?}"),
        };
        let stream = build(mixer, receiver)?;
        stream.play()?;
        Ok(Self {
            _stream: stream,
            commands,
            errors,
        })
    }
    pub fn try_command(&self, command: Command) -> Result<()> {
        validate_command(&command)?;
        self.commands
            .try_send(Message::Single(command))
            .map_err(|_| anyhow::anyhow!("audio command queue unavailable or full"))
    }
    /// Submit a complete frame of control changes in one queue entry.
    /// A full queue rejects the whole frame, allowing the caller to retry it.
    pub fn try_controls(&self, commands: Vec<Command>) -> Result<()> {
        send_controls(&self.commands, commands)
    }
    pub fn try_error(&self) -> Option<String> {
        self.errors.try_recv().ok()
    }
}

fn stream<T>(
    device: &cpal::Device,
    config: &cpal::StreamConfig,
    mut mixer: Mixer,
    receiver: Receiver<Message>,
    errors: mpsc::Sender<String>,
) -> Result<cpal::Stream>
where
    T: cpal::SizedSample + cpal::FromSample<f32>,
{
    let mut scratch = Vec::<f32>::new();
    let command_errors = errors.clone();
    Ok(device.build_output_stream(
        config,
        move |data: &mut [T], _| {
            while let Ok(message) = receiver.try_recv() {
                match message {
                    Message::Single(command) => {
                        if let Err(error) = mixer.apply(command) {
                            let _ = command_errors.send(error.to_string());
                        }
                    }
                    Message::Controls(commands) => {
                        for command in commands {
                            if let Err(error) = mixer.apply(command) {
                                let _ = command_errors.send(error.to_string());
                            }
                        }
                    }
                }
            }
            scratch.resize(data.len(), 0.0);
            mixer.render(&mut scratch);
            for (out, value) in data.iter_mut().zip(&scratch) {
                *out = T::from_sample(*value);
            }
        },
        move |error| {
            let _ = errors.send(error.to_string());
        },
        None,
    )?)
}

#[cfg(test)]
mod tests {
    use super::*;
    fn sample(values: &[f32], channels: u16, rate: u32) -> Arc<Sample> {
        Arc::new(Sample {
            channels,
            sample_rate: rate,
            samples: values.to_vec(),
        })
    }
    fn add(mixer: &mut Mixer, sample: Arc<Sample>, parameters: Parameters, looping: bool) {
        mixer
            .command(Command::Add {
                id: VoiceId(1),
                sample,
                parameters: Box::new(parameters),
                looping,
            })
            .unwrap();
        mixer
            .command(Command::Set {
                id: VoiceId(1),
                level: 0.5,
                inside: true,
                active: true,
            })
            .unwrap();
    }
    #[test]
    fn interpolates_loops_and_preserves_stereo_channels() {
        let mut mixer = Mixer::new(8, 2).unwrap();
        add(
            &mut mixer,
            sample(&[0.0, 1.0, 1.0, 0.0], 2, 4),
            Parameters::default(),
            true,
        );
        let mut output = [0.0; 10];
        mixer.render(&mut output);
        assert_eq!(output, [0.0, 1.0, 0.5, 0.5, 1.0, 0.0, 0.5, 0.5, 0.0, 1.0]);
    }
    #[test]
    fn applies_pitch_volume_viewpoint_and_one_shot_restart() {
        let mut mixer = Mixer::new(4, 1).unwrap();
        let parameters = Parameters {
            viewpoint: Some(2),
            pitch_envelope: Some(vec![[0.0, 1.0], [1.0, 2.0]]),
            volume_envelope: Some(vec![[0.0, 0.0], [1.0, 100.0]]),
            ..Default::default()
        };
        add(
            &mut mixer,
            sample(&[1.0, 1.0, 1.0, 1.0], 1, 4),
            parameters,
            false,
        );
        let mut output = [0.0; 2];
        mixer.render(&mut output);
        assert_eq!(output, [0.0, 0.0]);
        mixer
            .command(Command::Set {
                id: VoiceId(1),
                level: 0.5,
                inside: false,
                active: true,
            })
            .unwrap();
        mixer.render(&mut output);
        assert!((output[0] - 0.5f32.sqrt()).abs() < 1e-6);
        assert_eq!(output[1], 0.0);
        mixer
            .command(Command::Set {
                id: VoiceId(1),
                level: 0.5,
                inside: false,
                active: true,
            })
            .unwrap();
        mixer.render(&mut output);
        assert!((output[0] - 0.5f32.sqrt()).abs() < 1e-6);
    }
    #[test]
    fn validates_inputs_and_clamps_mixed_output() {
        let mut mixer = Mixer::new(8000, 1).unwrap();
        let bad = sample(&[f32::NAN], 1, 8000);
        assert!(
            mixer
                .command(Command::Add {
                    id: VoiceId(1),
                    sample: bad,
                    parameters: Box::new(Parameters::default()),
                    looping: true
                })
                .is_err()
        );
        for id in 0..2 {
            mixer
                .command(Command::Add {
                    id: VoiceId(id),
                    sample: sample(&[0.8], 1, 8000),
                    parameters: Box::new(Parameters::default()),
                    looping: true,
                })
                .unwrap();
            mixer
                .command(Command::Set {
                    id: VoiceId(id),
                    level: 1.0,
                    inside: true,
                    active: true,
                })
                .unwrap();
        }
        let mut output = [0.0; 2];
        mixer.render(&mut output);
        assert_eq!(output, [1.0, 1.0]);
    }

    #[test]
    fn validates_commands_before_the_audio_callback() {
        let invalid_sample = Command::Add {
            id: VoiceId(7),
            sample: sample(&[0.0, f32::NAN], 1, 8000),
            parameters: Box::new(Parameters::default()),
            looping: true,
        };
        assert!(validate_command(&invalid_sample).is_err());
        let invalid_envelope = Command::Add {
            id: VoiceId(7),
            sample: sample(&[0.0, 1.0], 1, 8000),
            parameters: Box::new(Parameters {
                pitch_envelope: Some(vec![[1.0, 1.0], [0.0, 1.0]]),
                ..Default::default()
            }),
            looping: true,
        };
        assert!(validate_command(&invalid_envelope).is_err());
        assert!(
            validate_command(&Command::Set {
                id: VoiceId(7),
                level: f64::NAN,
                inside: true,
                active: true,
            })
            .is_err()
        );
    }

    #[test]
    fn control_batches_are_validated_and_queued_as_one_frame() {
        let (sender, receiver) = mpsc::sync_channel(1);
        let set = |id, level| Command::Set {
            id: VoiceId(id),
            level,
            inside: true,
            active: true,
        };
        assert!(send_controls(&sender, vec![set(1, 0.5), set(2, f64::NAN)]).is_err());
        assert!(receiver.try_recv().is_err());
        assert!(send_controls(&sender, vec![Command::Remove(VoiceId(1))]).is_err());
        send_controls(&sender, vec![set(1, 0.5), set(2, 0.75)]).unwrap();
        assert!(send_controls(&sender, vec![set(3, 0.5)]).is_err());
        let Message::Controls(commands) = receiver.try_recv().unwrap() else {
            panic!("expected batch")
        };
        assert_eq!(commands.len(), 2);
        assert!(matches!(commands[0], Command::Set { id: VoiceId(1), .. }));
        assert!(matches!(commands[1], Command::Set { id: VoiceId(2), .. }));
        assert!(receiver.try_recv().is_err());
        send_controls(&sender, vec![set(3, 0.5)]).unwrap();
    }
}
