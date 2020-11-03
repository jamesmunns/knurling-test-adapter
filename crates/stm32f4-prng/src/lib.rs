#![no_std]

use rand::RngCore;
use stm32f4xx_hal::{
    stm32::ADC1,
    adc::{Adc, Temperature, Vref, config::Resolution},
    signature::{Uid, VrefCal, VtempCal30},
};
use rand_chacha::{
    ChaCha8Rng,
    rand_core::SeedableRng,
};
pub use stm32f4xx_hal::adc::config::SampleTime;

/// The sources of entropy to use
#[derive(Debug, Copy, Clone)]
pub enum EntropySources {
    TempOnly(SampleTime),
    VrefOnly(SampleTime),
    DevInfoAndTemp(SampleTime),
    DevInfoAndVref(SampleTime),
    TempAndVref(SampleTime),
    AllSources(SampleTime),
}

/// Configuration values for seeding the PRNG
#[derive(Debug, Copy, Clone)]
pub struct RngConfig {
    /// Minimum number of randomly generated words to discard
    ///
    /// Note: If this is less than 1024, 1024 numbers will be discarded
    pub min_init_cycles: u32,

    /// Maximum number of randomly generated words to discard
    ///
    /// Note: This must be greater than `min_init_cycles`, or the
    /// values will be swapped
    pub max_init_cycles: u32,

    /// The sources to use for runtime entropy gathering for the initial
    /// seed value. Options include:
    ///
    /// * Device Info, such as serial number and die position
    /// * Internal Temperature Sensor
    /// * Internal VRef measurement
    pub entropy_sources: EntropySources,
}

impl Default for RngConfig {
    fn default() -> Self {
        RngConfig {
            min_init_cycles: 8192,
            max_init_cycles: 16384,
            entropy_sources: EntropySources::AllSources(SampleTime::Cycles_480),
        }
    }
}

// Fill the device info into 16 bytes
fn fill_device_info(slice: &mut [u8; 16]) {
    let uid = Uid::get();
    let vtc30 = VtempCal30::get();
    let vrc = VrefCal::get();

    // 7 bytes (7/16)
    let lot = uid.lot_num().as_bytes();
    slice[..7].copy_from_slice(lot);

    // 1 byte (8/16)
    let waf = uid.waf_num();
    slice[7] = waf;

    // 2 bytes (10/16)
    let xpos = uid.x().to_ne_bytes();
    slice[8..10].copy_from_slice(&xpos);

    // 2 bytes (12/16)
    let ypos = uid.y().to_ne_bytes();
    slice[10..12].copy_from_slice(&ypos);

    // 2 bytes (14/16)
    let cal30 = vtc30.read().to_ne_bytes();
    slice[12..14].copy_from_slice(&cal30);

    // 2 bytes (16/16)
    let vrcal = vrc.read().to_ne_bytes();
    slice[14..16].copy_from_slice(&vrcal);
}

enum Source {
    Temp,
    Vref,
}

fn fill_adc_readings(adc: &mut Adc<ADC1>, source: Source, samples: SampleTime, bytes: &mut [u8]) {
    // TODO: Ensure all ADC reading aren't exactly the same?

    for byte in bytes.iter_mut() {
        // take the LSB of the 12 bit ADC reading for maximum noise
        *byte = match source {
            Source::Temp => adc.convert(&Temperature, samples),
            Source::Vref => adc.convert(&Vref, samples),
        } as u8;
    }
}

/// Create a new RNG, seeded with ADC data
pub fn seed_rng(adc: &mut Adc<ADC1>, config: RngConfig) -> ChaCha8Rng {
    adc.enable_temperature_and_vref();
    adc.set_resolution(Resolution::Twelve);

    let mut key = [0u8; 32];
    let mut bytes = [0u8; 16];

    match config.entropy_sources {
        EntropySources::TempOnly(sample_time) => {
            fill_adc_readings(adc, Source::Temp, sample_time, &mut key);
        }
        EntropySources::VrefOnly(sample_time) => {
            fill_adc_readings(adc, Source::Vref, sample_time, &mut key);
        }
        EntropySources::DevInfoAndTemp(sample_time) => {
            fill_device_info(&mut bytes);
            key[..16].copy_from_slice(&bytes);
            fill_adc_readings(adc, Source::Temp, sample_time, &mut key[16..]);
        }
        EntropySources::DevInfoAndVref(sample_time) => {
            fill_device_info(&mut bytes);
            key[..16].copy_from_slice(&bytes);
            fill_adc_readings(adc, Source::Vref, sample_time, &mut key[16..]);
        }
        EntropySources::TempAndVref(sample_time) => {
            fill_adc_readings(adc, Source::Temp, sample_time, &mut key[..16]);
            fill_adc_readings(adc, Source::Vref, sample_time, &mut key[16..]);
        }
        EntropySources::AllSources(sample_time) => {
            fill_device_info(&mut bytes);
            key[..16].copy_from_slice(&bytes);
            fill_adc_readings(adc, Source::Temp, sample_time, &mut key[16..24]);
            fill_adc_readings(adc, Source::Vref, sample_time, &mut key[24..]);
        }
    }

    let mut rng = ChaCha8Rng::from_seed(key);

    // Ensure we have min and max in the right order, and that we
    // have SOME amount of random entropy
    let min = config.min_init_cycles.min(config.max_init_cycles);
    let max = config.min_init_cycles.max(config.max_init_cycles);
    let end = (max - min).min(1024);

    let nop_cycles = min + (rng.next_u32() % end);

    for _ in 0..nop_cycles {
        let _ = rng.next_u32();
    }

    rng
}
