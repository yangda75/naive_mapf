use std::collections::{BinaryHeap, HashMap};
use std::cmp::Ordering;
use crate::algo::cbs::{GridPoint};

pub fn grid_search(start: &GridPoint, goal: &GridPoint, obstacles: &[GridPoint],
                   constraints: &[AstarConstraint], dim_x: isize, dim_y: isize,
) -> Option<GridAstarResult> {
    let mut open_set = BinaryHeap::new();
    let mut id: isize = 0;
    let mut search_node_cnt = 0;
    let h = heuristic(start, goal);
    let root = AstarNode {
        id,
        f: h,
        g: 0,
        h,
        point: start.clone(),
        parent: id,
    };
    open_set.push(root);
    let mut closed_set = HashMap::new();
    let mut final_node: AstarNode = AstarNode::dummy();
    let mut found = false;

    while !open_set.is_empty() {
        let n = open_set.pop()?;
        // println!("{:?}", &n);
        if &n.point == goal {
            final_node = n;
            found = true;
            break;
        }
        // generate neighbor astar nodes
        let neighbor_grid_points = get_neighbor_grid_points(&n.point);
        'outer: for (p, cost) in neighbor_grid_points {
            if p.x < 0 || p.y < 0 || p.x > dim_x - 1 || p.y > dim_y - 1 { continue; }
            if obstacles.contains(&p) { continue; }
            id += 1;
            let h_p = heuristic(&p, goal);
            let g_p = n.g + cost;

            // constraint
            for c in constraints {
                // println!("{:?}, {:?}, {:?}", &c, &p, &g_p);
                // constraint type
                if c.time2 == c.time1 {
                    if c.point1 == p && g_p == c.time1 {
                        continue 'outer;
                    }
                } else if c.point1 == n.point && c.point2 == p && c.time2 == g_p {
                    continue 'outer;
                }
            }
            let new_node = AstarNode {
                id,
                f: h_p + g_p,
                g: g_p,
                h: h_p,
                point: p.clone(),
                parent: n.id,
            };
            // println!("{:?}", &new_node);

            search_node_cnt += 1;
            open_set.push(new_node);
        }
        closed_set.insert(n.id, n);
    }
    // println!("{}", search_node_cnt);
    // build result
    if found {
        Some(GridAstarResult { cost: final_node.g, path: build_path(&final_node, &closed_set), search_node_cnt })
    } else {
        None
    }
}

fn build_path(n: &AstarNode, closed_set: &HashMap<isize, AstarNode>) -> Vec<GridPoint> {
    let mut path = vec![n.point.clone()];
    let mut p_id = n.parent;
    let mut node = n;
    while p_id != node.id {
        node = closed_set.get(&p_id).unwrap();
        p_id = node.parent;
        path.push(node.point.clone());
    }

    path.reverse();
    path
}

fn get_neighbor_grid_points(p: &GridPoint) -> Vec<(GridPoint, isize)> {
    vec![
        (p.clone(), 1), // wait penalty
        (GridPoint { x: p.x - 1, y: p.y }, 1),
        (GridPoint { x: p.x, y: p.y - 1 }, 1),
        (GridPoint { x: p.x + 1, y: p.y }, 1),
        (GridPoint { x: p.x, y: p.y + 1 }, 1),
    ]
}

fn heuristic(start: &GridPoint, goal: &GridPoint) -> isize {
    let x_diff = if start.x > goal.x {
        start.x - goal.x
    } else {
        goal.x - start.x
    };
    let y_diff = if start.y > goal.y {
        start.y - goal.y
    } else {
        goal.y - start.y
    };
    x_diff + y_diff
}

#[derive(Clone, Eq, PartialEq, Debug)]
pub struct AstarNode {
    id: isize,
    f: isize,
    g: isize,
    h: isize,
    point: GridPoint,
    parent: isize, // parent id
}

impl Ord for AstarNode {
    // min heap
    fn cmp(&self, other: &Self) -> Ordering {
        other.f.cmp(&self.f)
            .then_with(|| other.h.cmp(&self.h))
    }
}

impl PartialOrd for AstarNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl AstarNode {
    pub fn dummy() -> AstarNode {
        AstarNode {
            id: 0,
            f: 0,
            g: 0,
            h: 0,
            point: GridPoint { x: 0, y: 0 },
            parent: 0,
        }
    }
}

#[derive(Debug)]
pub struct GridAstarResult {
    pub path: Vec<GridPoint>,
    pub cost: isize,
    pub search_node_cnt: usize,
}

#[derive(Eq, PartialEq, Debug)]
pub struct AstarConstraint {
    pub(crate) point1: GridPoint,
    pub(crate) point2: GridPoint,
    pub(crate) time1: isize,
    pub(crate) time2: isize,
}
