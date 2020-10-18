use plotters::prelude::*;

const COLORS: [RGBColor; 3] = [RED, BLUE, GREEN];

pub fn auto_graph(
    series: Vec<(f64, Vec<f64>)>,
    output_path: &str,
    title: &str,
    y_desc: &str,
    x_desc: &str,
    y_modes: Vec<&str>,
    resolution: (u32, u32)
) -> Result<(), Box<dyn std::error::Error>> {
    let ((x_min, x_max), (y_min, y_max)) = series.iter().fold(
        (
            (std::f64::MAX, std::f64::MIN),
            (std::f64::MAX, std::f64::MIN),
        ),
        |((x_min, x_max), (y_min, y_max)), b| {
            ((x_min.min(b.0), x_max.max(b.0)), (y_min.min(min(&b.1)), y_max.max(max(&b.1))))
        },
    );

    let root = BitMapBackend::new(output_path, resolution).into_drawing_area();
    root.fill(&WHITE)?;

    let mut chart = ChartBuilder::on(&root)
        .caption(title, ("sans-serif", 35))
        .margin(30)
        .margin_top(20)
        .x_label_area_size(40)
        .y_label_area_size(50)
        .build_cartesian_2d(
            x_min..x_max,
            ((1. - y_min.signum() * 0.05) * y_min)..((1. + y_max.signum() * 0.05) * y_max),
        )?;

    chart
        .configure_mesh()
        .axis_desc_style(("sans-serif", 15))
        .y_desc(y_desc)
        .x_desc(x_desc)
        .draw()?;

    for (i, &mode) in y_modes.iter().enumerate() {
        chart
            .draw_series(LineSeries::new(
                series.iter().map(|(x, ys)| (*x, ys[i])),
                &COLORS[i % COLORS.len()],
            ))?
            .label(mode)
            .legend(move |(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &COLORS[i % COLORS.len()]));
    }

    chart
        .configure_series_labels()
        .background_style(&WHITE.mix(0.8))
        .border_style(&BLACK)
        .draw()?;

    Ok(())
}

fn max(series: &[f64]) -> f64 {
    series.into_iter().fold(std::f64::MIN, |acc, &x| acc.max(x))
}

fn min(series: &[f64]) -> f64 {
    series.into_iter().fold(std::f64::MAX, |acc, &x| acc.min(x))
}
