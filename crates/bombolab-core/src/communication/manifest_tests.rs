use super::manifest::{is_v2_line, parse_manifest_file};

#[test]
fn parsea_un_manifest_valido() {
    let content = "# demo\nMANIFEST 2 100000\nSAMPLE 1500 1500 1500 1500 1500 1472 0\nSAMPLE 1600 1500 1500 1500 1500 1472 100000\n";
    let lines = parse_manifest_file(content).unwrap();
    assert_eq!(lines.len(), 3);
    assert!(lines[1].starts_with("SAMPLE "));
}

#[test]
fn rechaza_primer_dt_distinto_de_cero() {
    let content = "SAMPLE 1500 1500 1500 1500 1500 1472 50000\n";
    assert!(parse_manifest_file(content).is_err());
}

#[test]
fn rechaza_dt_cero_intermedio() {
    let content =
        "SAMPLE 1500 1500 1500 1500 1500 1472 0\nSAMPLE 1600 1500 1500 1500 1500 1472 0\n";
    assert!(parse_manifest_file(content).is_err());
}

#[test]
fn rechaza_linea_desconocida() {
    let content = "SAMPLE 1500 1500 1500 1500 1500 1472 0\nGARBAGE 1 2\n";
    assert!(parse_manifest_file(content).is_err());
}

#[test]
fn rechaza_manifest_sin_samples() {
    let content = "MANIFEST 2 100000\n";
    assert!(parse_manifest_file(content).is_err());
}

#[test]
fn rechaza_sample_malformada() {
    let content = "SAMPLE 1500 1500 1500\n";
    assert!(parse_manifest_file(content).is_err());
}

#[test]
fn reconoce_lineas_v2() {
    assert!(is_v2_line("HELLO 2"));
    assert!(is_v2_line("EXECUTE"));
    assert!(is_v2_line("T 50000 1500 1500 1500 1500 1500 1472"));
    assert!(!is_v2_line("1500,1500,1500,1500,1500,1472"));
}
