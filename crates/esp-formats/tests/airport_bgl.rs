use esp_formats::bgl::Bgl;

#[test]
#[ignore = "requires an airport BGL in OPENESP_AIRPORT_BGL"]
fn loads_airport_and_runways_from_real_bgl() {
    let file = std::env::var_os("OPENESP_AIRPORT_BGL").expect("set OPENESP_AIRPORT_BGL");
    let bytes = std::fs::read(file).unwrap();
    let airports = Bgl::parse(&bytes).unwrap().airports().unwrap();
    eprintln!(
        "airports: {:?}",
        airports
            .iter()
            .map(|airport| (&airport.ident, airport.runways.len()))
            .collect::<Vec<_>>()
    );
    assert!(!airports.is_empty());
    assert!(airports.iter().any(|airport| !airport.runways.is_empty()));
    assert!(
        airports
            .iter()
            .flat_map(|airport| &airport.runways)
            .all(|runway| {
                runway.latitude.abs() <= 90.0
                    && runway.longitude.abs() <= 180.0
                    && runway.length_m.is_finite()
                    && runway.length_m > 0.0
                    && runway.width_m.is_finite()
                    && runway.width_m > 0.0
                    && runway.heading_deg.is_finite()
            })
    );
    if let Some(airport) = airports.iter().find(|airport| airport.ident == "KLAX") {
        assert_eq!(airport.runways.len(), 4);
        assert!(
            airport
                .runways
                .iter()
                .any(|runway| runway.primary_number == 7 && runway.length_m > 3900.0)
        );
    }
}
