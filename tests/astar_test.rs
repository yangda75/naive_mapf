use naive_mapf::algo::cbs::GridPoint;
use naive_mapf::algo::astar::grid_search;

#[test]
fn astar_test() {
    let start = GridPoint { x: 0, y: 0 };
    let goal = GridPoint { x: 10, y: 10 };
    let result = grid_search(&start, &goal, &[], &[], 100, 100).unwrap();
    // println!("{}", result.cost);
    // println!("{:#?}",result.path);
    assert_eq!(result.cost, 20);
}