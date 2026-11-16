//! Raw access to a log's messages, for checking ArduTune's analyses against
//! the data itself (the AI assistant's log tools): which message types there
//! are, and the rows of one type, filtered, thinned or summarised.

use serde::Deserialize;
use serde_json::{json, Value};

use crate::log::{Column, Log};

/// Rows returned at most, however many are asked for.
const MAX_ROWS: usize = 5000;

/// Every message type in the log: rows, fields and the instance field (I,
/// Instance, C…) when one type is logged per sensor or motor.
pub fn types(log: &Log) -> Value {
    let mut out: Vec<Value> = log
        .tables()
        .filter(|t| !t.is_empty())
        .map(|t| {
            let span = t.time_s().and_then(|s| Some((*s.first()?, *s.last()?)));
            json!({
                "type": t.name,
                "rows": t.len(),
                "fields": t.labels,
                "time_s": span.map(|(a, b)| [round(a), round(b)]),
            })
        })
        .collect();
    out.sort_by(|a, b| a["type"].as_str().cmp(&b["type"].as_str()));
    json!({ "types": out, "parse": log.stats })
}

#[derive(Debug, Deserialize, Default)]
pub struct Query {
    #[serde(rename = "type")]
    pub kind: String,
    /// Fields to return; all when empty.
    #[serde(default)]
    pub fields: Vec<String>,
    /// Only rows where each of these fields equals the value (instance, motor…).
    #[serde(default, rename = "where")]
    pub filter: std::collections::BTreeMap<String, f64>,
    pub from_s: Option<f64>,
    pub to_s: Option<f64>,
    /// Rows at most (evenly thinned beyond that); default 200.
    pub max_rows: Option<usize>,
    /// Instead of rows: count, min, max, mean and standard deviation per field.
    #[serde(default)]
    pub stats: bool,
}

fn round(v: f64) -> f64 {
    (v * 1e6).round() / 1e6
}

fn cell(c: &Column, i: usize) -> Value {
    match c {
        Column::Num(v) => json!(round(v[i])),
        Column::Text(v) => json!(v[i]),
        Column::I16x32(v) => json!(v[i].to_vec()),
    }
}

/// The rows of one message type matching `q`, or their statistics.
pub fn query(log: &Log, q: &Query) -> Result<Value, String> {
    let t = log.table(&q.kind).ok_or_else(|| format!("This log has no {} messages (see log_message_types).", q.kind))?;
    let fields: Vec<String> = if q.fields.is_empty() { t.labels.clone() } else { q.fields.clone() };
    let index: Vec<usize> = fields
        .iter()
        .map(|f| t.labels.iter().position(|l| l == f).ok_or_else(|| format!("{} has no field {f}; it has {}.", q.kind, t.labels.join(", "))))
        .collect::<Result<_, _>>()?;
    let time = t.time_s();
    let filters: Vec<(&[f64], f64)> = q
        .filter
        .iter()
        .map(|(f, v)| t.num(f).map(|col| (col, *v)).ok_or_else(|| format!("{} has no numeric field {f} to filter on.", q.kind)))
        .collect::<Result<_, _>>()?;
    let matched: Vec<usize> = (0..t.len())
        .filter(|&i| filters.iter().all(|(col, v)| (col[i] - v).abs() < 1e-9))
        .filter(|&i| match (&time, q.from_s, q.to_s) {
            (Some(ts), from, to) => from.is_none_or(|a| ts[i] >= a) && to.is_none_or(|b| ts[i] <= b),
            _ => true,
        })
        .collect();

    if q.stats {
        let stats: Vec<Value> = fields
            .iter()
            .zip(&index)
            .filter_map(|(name, &k)| {
                let Column::Num(v) = &t.columns[k] else { return None };
                let xs: Vec<f64> = matched.iter().map(|&i| v[i]).filter(|x| x.is_finite()).collect();
                if xs.is_empty() {
                    return Some(json!({ "field": name, "count": 0 }));
                }
                let n = xs.len() as f64;
                let mean = xs.iter().sum::<f64>() / n;
                let sd = (xs.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / n).sqrt();
                let (min, max) = xs.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), x| (a.min(*x), b.max(*x)));
                Some(json!({ "field": name, "count": xs.len(), "min": round(min), "max": round(max), "mean": round(mean), "std": round(sd) }))
            })
            .collect();
        return Ok(json!({ "type": q.kind, "total_rows": t.len(), "matched_rows": matched.len(), "stats": stats }));
    }

    let limit = q.max_rows.unwrap_or(200).clamp(1, MAX_ROWS);
    let step = matched.len().div_ceil(limit).max(1);
    let rows: Vec<Value> = matched
        .iter()
        .step_by(step)
        .map(|&i| {
            let mut row = Vec::with_capacity(fields.len() + 1);
            if let Some(ts) = &time {
                row.push(json!(round(ts[i])));
            }
            row.extend(index.iter().map(|&k| cell(&t.columns[k], i)));
            Value::Array(row)
        })
        .collect();
    let mut columns: Vec<&str> = Vec::new();
    if time.is_some() {
        columns.push("time_s");
    }
    columns.extend(fields.iter().map(String::as_str));
    Ok(json!({
        "type": q.kind,
        "columns": columns,
        "total_rows": t.len(),
        "matched_rows": matched.len(),
        "every_nth": step,
        "rows": rows,
    }))
}

/// The parameters recorded in the log, optionally only those whose name
/// contains `search`.
pub fn params(log: &Log, search: Option<&str>) -> Value {
    let q = search.map(str::to_uppercase);
    let out: serde_json::Map<String, Value> =
        log.params().iter().filter(|(n, _)| q.as_ref().is_none_or(|q| n.contains(q.as_str()))).map(|(n, v)| (n.clone(), json!(v))).collect();
    json!({ "count": out.len(), "params": out })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::synth::{sample_flight, Preset};

    #[test]
    fn types_rows_stats_and_params() {
        let log = Log::from_bytes(&sample_flight(Preset::Ready));
        let t = types(&log);
        let rate = t["types"].as_array().unwrap().iter().find(|x| x["type"] == "RATE").expect("RATE messages");
        assert!(rate["rows"].as_u64().unwrap() > 1000 && rate["fields"].as_array().unwrap().iter().any(|f| f == "R"));

        let q = Query {
            kind: "RATE".into(),
            fields: vec!["RDes".into(), "R".into()],
            from_s: Some(10.0),
            to_s: Some(20.0),
            max_rows: Some(50),
            ..Default::default()
        };
        let r = query(&log, &q).unwrap();
        assert_eq!(r["columns"], json!(["time_s", "RDes", "R"]));
        let rows = r["rows"].as_array().unwrap();
        assert!(rows.len() <= 50 && !rows.is_empty());
        assert!(rows.iter().all(|row| (10.0..=20.0).contains(&row[0].as_f64().unwrap())));

        let s = query(&log, &Query { stats: true, ..q }).unwrap();
        assert_eq!(s["stats"][1]["field"], json!("R"));
        assert!(s["stats"][1]["count"].as_u64().unwrap() > 0);

        assert!(query(&log, &Query { kind: "NOPE".into(), ..Default::default() }).unwrap_err().contains("no NOPE"));
        assert!(query(&log, &Query { kind: "RATE".into(), fields: vec!["X".into()], ..Default::default() }).unwrap_err().contains("no field X"));

        let p = params(&log, Some("ins_hntch"));
        assert!(p["count"].as_u64().unwrap() > 0 && p["params"].as_object().unwrap().keys().all(|k| k.contains("INS_HNTCH")));
    }
}
