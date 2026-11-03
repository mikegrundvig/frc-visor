//! NAPA, through Visor's wrapper, on the rendered ThriftyCam frames.

mod common;

use visor::napa::Detector;

#[test]
fn every_rendered_tag_is_found_where_it_is() {
    let mut detector = Detector::new(16).unwrap();
    for name in common::SCENES {
        let scene = common::scene(name);
        let found = detector.detect(&scene.image).unwrap();
        let mut ids: Vec<i32> = found.iter().map(|d| d.id).collect();
        ids.sort();
        let mut expected: Vec<i32> = scene.tags.iter().map(|t| t.id).collect();
        expected.sort();
        assert_eq!(ids, expected, "{}: the tags found", scene.name);
        for known in &scene.tags {
            let detection = found.iter().find(|d| d.id == known.id).unwrap();
            // Same order and convention as the renderer's corners, to a tenth of a pixel. The
            // order matters: the robot solves poses from it.
            let error = common::worst_corner(&detection.corners, &known.corners);
            assert!(
                error < 0.1,
                "{}: tag {} corners off by {error:.3} px",
                scene.name,
                known.id
            );
        }
    }
}

#[test]
fn a_larger_minimum_misses_the_far_tag() {
    // two-tags has a tag at 3 m and one at 8 m; the 8 m one's square is about 20 px across.
    let scene = common::scene("two-tags");
    let mut detector = Detector::new(64).unwrap();
    let ids: Vec<i32> = detector
        .detect(&scene.image)
        .unwrap()
        .iter()
        .map(|d| d.id)
        .collect();
    assert_eq!(ids, [3]);
}
