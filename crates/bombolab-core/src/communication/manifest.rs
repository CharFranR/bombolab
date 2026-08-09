const V2_KEYWORDS: [&str; 8] = [
    "HELLO",
    "MANIFEST",
    "SAMPLE",
    "END_UPLOAD",
    "EXECUTE",
    "STOP",
    "STATUS",
    "T ",
];

pub fn is_v2_line(line: &str) -> bool {
    V2_KEYWORDS.iter().any(|k| line.starts_with(k))
}

fn parse_u32_field(s: &str) -> Result<(u32, &str), String> {
    let t = s.trim_start();
    let end = t.find(|c: char| !c.is_ascii_digit()).unwrap_or(t.len());
    if end == 0 {
        return Err("se esperaba un número".into());
    }
    let num: u32 = t[..end]
        .parse()
        .map_err(|_| format!("número demasiado grande: {}", &t[..end]))?;
    Ok((num, &t[end..]))
}

fn parse_sample(line: &str) -> Result<u32, String> {
    let rest = &line["SAMPLE ".len()..];
    let mut cursor = rest;
    for _ in 0..6 {
        let (_, after) = parse_u32_field(cursor).map_err(|e| format!("SAMPLE inválida: {e}"))?;
        cursor = after.trim_start();
    }
    let (dt, after) = parse_u32_field(cursor).map_err(|e| format!("SAMPLE inválida: {e}"))?;
    if !after.trim().is_empty() {
        return Err("SAMPLE con campos de más".into());
    }
    Ok(dt)
}

fn parse_manifest(line: &str) -> Result<(), String> {
    let rest = &line["MANIFEST ".len()..];
    let (_, after) = parse_u32_field(rest).map_err(|e| format!("MANIFEST inválida: {e}"))?;
    let (_, after) = parse_u32_field(after).map_err(|e| format!("MANIFEST inválida: {e}"))?;
    if !after.trim().is_empty() {
        return Err("MANIFEST con campos de más".into());
    }
    Ok(())
}

pub fn parse_manifest_file(content: &str) -> Result<Vec<String>, String> {
    let mut lines: Vec<String> = Vec::new();
    let mut sample_count = 0usize;
    let mut saw_first = false;
    for raw in content.lines() {
        let line = raw.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        if line.starts_with("SAMPLE ") {
            let dt = parse_sample(line)?;
            if !saw_first {
                if dt != 0 {
                    return Err("el primer dt debe ser 0".into());
                }
                saw_first = true;
            } else if dt == 0 {
                return Err("dt debe ser > 0".into());
            }
            sample_count += 1;
        } else if line.starts_with("MANIFEST ") {
            parse_manifest(line)?;
        } else if !is_v2_line(line) {
            return Err(format!("línea desconocida: {line}"));
        }
        lines.push(line.to_string());
    }
    if sample_count == 0 {
        return Err("el manifest no contiene SAMPLEs".into());
    }
    Ok(lines)
}
