use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};
use crate::algo::astar::{grid_search, AstarConstraint};

///
/// Conflict Based Search
pub struct Cbs {
    env: GridEnvironment,
}

// TODO maybe represent conflicts as a set of needed constraints
#[derive(Debug)]
pub struct CbsConflict {
    agent1: usize,
    agent2: usize,
    point1: GridPoint,
    point2: GridPoint,
    time1: usize,
    time2: usize,
}

#[derive(Clone, Eq, PartialEq)]
pub struct CbsConstraint {
    agent: usize,
    point1: GridPoint,
    point2: GridPoint,
    time1: usize,
    time2: usize,
}

/// High Level Node
#[derive(Clone)]
pub struct CbsHNode {
    cost: Vec<isize>,
    cost_sum: isize,
    constraints: HashMap<usize, Vec<CbsConstraint>>,
    path_table: PathTable,
}

impl PartialEq for CbsHNode {
    fn eq(&self, other: &Self) -> bool {
        self.cost_sum.eq(&other.cost_sum)
    }
}

impl Eq for CbsHNode {}

impl Ord for CbsHNode {
    fn cmp(&self, other: &Self) -> Ordering {
        other.cost_sum.cmp(&self.cost_sum)
    }
}

impl PartialOrd for CbsHNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

pub struct GridEnvironment {
    pub number_of_agents: usize,
    pub start_points: Vec<GridPoint>,
    pub goal_points: Vec<GridPoint>,
    // x size
    pub dim_x: isize,
    // y size
    pub dim_y: isize,
    pub obstacles: Vec<GridPoint>,
}

#[derive(Clone, Eq, PartialEq, Debug)]
pub struct GridPoint {
    pub x: isize,
    pub y: isize,
}

type PathTable = Vec<Vec<GridPoint>>;

#[derive(Debug)]
pub struct CbsResult {
    pub path_table: PathTable,
    pub conflict_cnt: usize,
    pub high_level_node_cnt: usize,
    pub low_level_node_cnt: usize,
    pub makespan: usize,
    pub cost_sum: isize,
}

impl Cbs {
    pub fn new(env: GridEnvironment) -> Cbs {
        Cbs {
            env,
        }
    }

    pub fn search(&mut self) -> Option<CbsResult> {
        let mut high_level_node_cnt = 0;
        let mut low_level_node_cnt = 0;
        let mut conflict_cnt = 0;
        // initial high level node
        let initial_node = CbsHNode::get_initial(&self.env)?;

        let mut final_node = CbsHNode::dummy();
        let mut open_set = BinaryHeap::new();

        open_set.push(initial_node);
        let mut found = false;

        while !open_set.is_empty() {
            let n = open_set.pop()?;
            // check conflicts
            let conflict = n.find_first_conflict();
            println!("conflict: {:?}", conflict);
            match conflict {
                Some(conf) => {
                    conflict_cnt += 1;
                    let (mut left, mut right) = n.split_on_conflict(conf);
                    if let Some(cnt) = left.calc_path_table(&self.env) {
                        low_level_node_cnt += cnt;
                        open_set.push(left);
                        high_level_node_cnt += 1;
                    }
                    if let Some(cnt) = right.calc_path_table(&self.env) {
                        low_level_node_cnt += cnt;
                        open_set.push(right);
                        high_level_node_cnt += 1;
                    }
                }
                None => {
                    // no conflict, found solution
                    final_node = n.clone();
                    found = true;
                    break;
                }
            }
        }
        println!("{}", high_level_node_cnt);
        let mut makespan = 0;
        for p in &final_node.path_table {
            if p.len() > makespan {
                makespan = p.len();
            }
        }

        if found {
            Some(CbsResult {
                path_table: final_node.path_table.clone(),
                makespan,
                high_level_node_cnt,
                low_level_node_cnt,
                conflict_cnt,
                cost_sum: final_node.cost_sum,
            })
        } else {
            None
        }
    }
}

impl CbsHNode {
    pub fn get_initial(env: &GridEnvironment) -> Option<CbsHNode> {
        let mut node = Self::dummy();

        node.calc_path_table(env)?;

        Some(node)
    }

    pub fn dummy() -> CbsHNode {
        CbsHNode {
            cost: vec![],
            cost_sum: 0,
            constraints: Default::default(),
            path_table: vec![],
        }
    }

    pub fn find_first_conflict(&self) -> Option<CbsConflict> {
        let mut max_len = 0;
        for p in &self.path_table {
            if p.len() > max_len {
                max_len = p.len();
            }
        }

        let mut last_configuration: HashMap<usize, &GridPoint> = HashMap::new();
        for i in 0..max_len {
            let mut current_configuration: HashMap<usize, &GridPoint> = HashMap::new();
            for a in 0..self.path_table.len() {
                let path = &self.path_table[a];
                if path.len() > i {
                    let p = &path[i];

                    current_configuration.insert(a, p);
                }
            }
            for (agent, point) in &current_configuration {
                // same location conflict
                for (agent2, point2) in &current_configuration {
                    if point2 == point && agent2 != agent {
                        return Some(CbsConflict {
                            agent1: *agent,
                            agent2: *agent2,
                            point1: (*point).clone(),
                            point2: (*point).clone(),
                            time1: i,
                            time2: i,
                        });
                    }
                }
            }

            // [agent, (point1, point2)]
            let mut points_by_agent: HashMap<usize, (&GridPoint, &GridPoint)> = HashMap::new();
            for (agent, point) in &last_configuration {
                if current_configuration.contains_key(agent) {
                    points_by_agent.insert(*agent, (*point, current_configuration[agent]));
                }
            }
            // exchange conflict
            for agent1 in points_by_agent.keys() {
                for agent2 in points_by_agent.keys() {
                    if agent2 != agent1 {
                        let p1 = points_by_agent[agent1];
                        let p2 = points_by_agent[agent2];
                        if p1.0 == p2.1 && p1.1 == p2.0 {
                            return Some(CbsConflict {
                                agent1: *agent1,
                                agent2: *agent2,
                                point1: (*p1.0).clone(), // agent1,time1,point1. agent2,time2,point1
                                point2: (*p1.1).clone(),
                                time1: i - 1,
                                time2: i,
                            });
                        }
                    }
                }
            }

            last_configuration = current_configuration
        }

        None
    }

    fn add_constraint(&mut self, constraint: CbsConstraint) {
        let e = self.constraints.entry(constraint.agent)
            .or_insert_with(Vec::new);
        if !e.contains(&constraint) {
            e.push(constraint);
        }
    }

    pub fn split_on_conflict(&self, conflict: CbsConflict) -> (Self, Self) {
        let mut left = self.clone();
        let mut right = self.clone();

        // conflict type
        if conflict.time2 == conflict.time1 {
            // same location

            left.add_constraint(CbsConstraint {
                agent: conflict.agent1,
                point1: conflict.point1.clone(),
                point2: conflict.point1.clone(),
                time1: conflict.time1,
                time2: conflict.time1,
            });

            right.add_constraint(CbsConstraint {
                agent: conflict.agent2,
                point1: conflict.point1.clone(),
                point2: conflict.point1.clone(),
                time1: conflict.time1,
                time2: conflict.time1,
            });
        } else {
            left.add_constraint(CbsConstraint {
                agent: conflict.agent1,
                point1: conflict.point1.clone(),
                point2: conflict.point2.clone(),
                time1: conflict.time1,
                time2: conflict.time2,
            });

            right.add_constraint(CbsConstraint {
                agent: conflict.agent2,
                point1: conflict.point2.clone(),
                point2: conflict.point1.clone(),
                time1: conflict.time1,
                time2: conflict.time2,
            })
        }

        (left, right)
    }

    pub fn calc_path_table(&mut self, env: &GridEnvironment) -> Option<usize> {
        let mut path_table = PathTable::new();
        let mut cost = Vec::new();
        // println!("calc path table");
        let mut low_level_node_cnt = 0;
        for i in 0..env.number_of_agents {
            let mut constraints = vec![];
            if self.constraints.contains_key(&i) {
                for cons in &self.constraints[&i] {
                    constraints.push(AstarConstraint {
                        point1: cons.point1.clone(),
                        point2: cons.point2.clone(),
                        time1: cons.time1 as isize,
                        time2: cons.time2 as isize,
                    })
                }
            }
            let astar_result = grid_search(
                &env.start_points[i], &env.goal_points[i],
                &env.obstacles, &constraints, env.dim_x, env.dim_y)?;
            // println!("{}, {:?}", i, astar_result);
            low_level_node_cnt += astar_result.search_node_cnt;
            path_table.push(astar_result.path);
            cost.push(astar_result.cost);
        }
        let cost_sum: isize = cost.iter().sum();

        self.path_table = path_table;
        self.cost = cost;
        self.cost_sum = cost_sum;

        Some(low_level_node_cnt)
    }
}