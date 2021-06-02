use naive_mapf::algo::cbs::{GridEnvironment, GridPoint, Cbs};

#[test]
fn cbs_test() {
    let env = GridEnvironment {
        number_of_agents: 2,
        start_points: vec![GridPoint { x: 0, y: 0 }, GridPoint { x: 10, y: 0 }],
        goal_points: vec![GridPoint { x: 10, y: 0 }, GridPoint { x: 0, y: 0 }],
        dim_x: 100,
        dim_y: 100,
        obstacles: vec![],
    };
    let mut cbs = Cbs::new(env);
    let result = cbs.search().unwrap();

    // println!("{:?}", result);
    assert_eq!(result.cost_sum, 22)
}