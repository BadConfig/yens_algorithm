use std::io::{self,Read};
use std::io::prelude::*;
use rand;
use rand::Rng;

fn read_vec() -> String {
    let mut val = String::new();
    io::stdin()
        .read_line(&mut val)
        .unwrap();
    val
}

struct Graph {
    edges_count: usize,
    matr: Vec<Vec<isize>>,
    rm_verts: Vec<usize>,
    rm_edges: Vec<(usize,usize)>,
}

enum Alg {
    Dijkstra,
    Ford,
}

impl Graph {
    //создает новый граф из количества вершин
    pub fn new(edges_count: usize) -> Self {
        Graph {
            edges_count: edges_count,
            matr: vec![vec![0;edges_count];edges_count],
            rm_verts: Vec::new(),
            rm_edges: Vec::new(),
        }
    }

    //получает в графе все вершины инцедентные входной
    pub fn get_row(&self, vertex: usize) -> Vec<(usize,isize)> {
        use std::ops::Index;
        self.matr
            .index(vertex)
            .into_iter()
            .map(|val| *val)
            .enumerate()
            .filter(|(num,val)| *val != 0)
            .filter(|(num,val)| !self.rm_edges.contains(&(vertex,*num)))
            .filter(|(num,val)| !self.rm_verts.contains(&vertex))
            .filter(|(num,val)| !self.rm_verts.contains(&num))
            .collect()
    }

    // получает все ребра графа
    pub fn get_all_edges(&self) -> Vec<(usize,usize,isize)> {
        let mut acc = Vec::new();
        for i in 0..self.edges_count {
            for j in 0..self.edges_count {
                if self.rm_verts.contains(&i) {
                    continue;
                }
                if self.rm_verts.contains(&j) {
                    continue;
                }
                if self.rm_edges.contains(&(i,j)) {
                    continue;
                }
                if self.matr[j][i] == 0 {
                    continue;
                }
                acc.push((i,j,self.matr[i][j]));
            }
        }
        acc
    }

    // удаляет ребро графа
    pub fn remove_edge(&mut self, from: usize, to: usize) {
        self.rm_edges.push((from,to));
        self.rm_edges.push((to,from));
    }

    //удаляет вершину графа 
    pub fn remove_vertex(&mut self, v: usize) {
        self.rm_verts.push(v);
    }

    // восстанавливает все удаленные ребра и вершины
    pub fn restore_removed(&mut self) {
        self.rm_verts.clear();
        self.rm_edges.clear();
    }

    //добавляет в граф ребро
    pub fn add_edge(
        &mut self,
        from: usize,
        to: usize,
        weight: isize,
    ) {
        self.matr[from][to] = weight;
        self.matr[to][from] = weight;
    }

    // реализация алгоритма дейкстры
    pub fn dijkstra(&self,from: usize, to: usize) -> (usize,Vec<usize>) {
        let mut distances = vec![usize::MAX;self.edges_count];
        distances[from] = 0;
        let mut previous = vec![None;self.edges_count];
        if from == to {
           return (0,Vec::new());
        }

        let mut queue = Vec::new();
        queue.push((0usize,from));
        while let Some(el) = queue.pop() {
            for (vertex,weight) in self.get_row(el.1) {
                let weight = weight as usize;
                if distances[vertex] == usize::MAX ||
                    weight + distances[el.1] < distances[vertex] {
                        distances[vertex] = weight + distances[el.1];
                        previous[vertex] = Some(el.1);
                        queue.push((distances[vertex],vertex));
                }
            }
            queue.sort_by(|a,b|b.0.cmp(&a.0));
        }
        let mut result = Vec::new();
        let mut current = to;
        while let Some(vertex) = previous[current] {
            result.push(vertex);
            current = vertex as usize;
        }
        result.reverse();
        result.push(to);
        (distances[to],result)
    }

    // реализация алгоритма белмана форда
    pub fn ford(&self, from: usize, to: usize) -> (usize,Vec<usize>) {
        let mut distances = vec![isize::MAX;self.edges_count];
        distances[from] = 0;
        let mut previous = vec![None;self.edges_count];

        for _ in 0..self.edges_count {
            for (f,t,w) in self.get_all_edges() {
                if distances[f] != isize::MAX && w + distances[f] < distances[t] {
                        distances[t] = w + distances[f];
                        previous[t] = Some(f);
                        
                }
            }
        }

        for (f,t,w) in self.get_all_edges() {
            if distances[f] != isize::MAX && w + distances[f] < distances[t] {
                        println!("neg sycle");
                        panic!();
            }
        }
        let mut result = Vec::new();
        let mut current = to;
        while let Some(vertex) = previous[current] {
            result.push(vertex);
            current = vertex;
        }
        result.reverse();
        result.push(to);
        (distances[to] as usize,result)
    }

    // получение длины пути в графе
    pub fn get_path_len(&self,path: Vec<usize>) -> usize {
        let mut acc = 0;
        if path.len() == 2 {
            return self.matr[path[0]][path[1]] as usize;
        }
        for i in 0..path.len()-1 {
            acc += self.matr[path[i]][path[i+1]];
        }
        acc as usize
    }

    // реализация алгоритма йенна 
    pub fn yens_alg(
        &mut self, 
        // исходная вершина
        from: usize, 
        // конечная вершина
        to: usize, 
        // к - кол-во кратчайших путей
        k: usize,
        // вспомогательный алгоритм для поиска кратчайшего пути
        path_alg: Alg,
    ) -> Vec<(Vec<usize>,usize)> {
        // если мы хотим попасть из вершины в нее же саму то возвращаем 0
        if from == to {
            return vec![(Vec::new(),0)];
        }
        // получаем кратчайший маршрут в графе и его длинну
        let (len,path) = match path_alg {
            Alg::Dijkstra => self.dijkstra(from, to),
            Alg::Ford => self.ford(from, to),
        };

        // проверяем что маршрут действительно доходит до нужной точки
        if !path.contains(&to) {
            panic!();
        }
        // добавляем маршрут в массив к кратчайших маршрутов
        let mut result: Vec<(Vec<usize>,usize)> = Vec::new();
        result.push((path,len));
        // инициализируем массив кандидатов
        let mut candidates: Vec<(Vec<usize>,usize)> = Vec::new();
        // выполняем алгоритм к-1 раз
        for mk in 1..k {
            println!("k == {}",mk);
            // берем последний добавленный в массив результатов путь
            let lst = &result
                .last()
                .unwrap().0;
            let t = lst.len()-1;
            // запускаем цикл переменная которого идет по кол-ву элементов
            // в последнем массиве пути без последнего
            for i in 0..t {
                // текущий узел ответвления - итый
                let spur_node = lst[i];
                // текущий корневой путь - путь до узла ответвления включая него
                let root_path = &lst[..i+1];
                // проходимся по массиву кратчайших путей и удаляем все ребра, которые
                // строят посещенные ответвления
                for p in &result {
                    if p.0.len() > i && root_path == &p.0[..i+1] {
                        self.remove_edge(p.0[i], p.0[i+1]); 
                    }
                }
                // удаляем все вершины основного пути из графа
                for v in root_path {
                    if *v == spur_node {
                        continue;
                    }
                    self.remove_vertex(*v);
                }

                //  выполняем поиск кратчайшего маршрута в видоизмененном графе
                let (spur_path_len,spur_path) = match path_alg {
                    Alg::Dijkstra => self.dijkstra(spur_node, to),
                    Alg::Ford => self.ford(spur_node, to),
                };
                // удаляем spur_node из корневого элемента
                let root_path = &root_path[..root_path.len()-1];
                // конкатенируем исходный и найденный путь, получаем кандидата
                let res_path = vec![root_path,&spur_path].concat();
                // находим длинну пути кандидата
                let len = self.get_path_len(res_path.clone());
                // если такого пути еще небыло - добавлем в массив кандидатов
                if !candidates.contains(&(res_path.clone(),len)) &&
                    !result.contains(&(res_path.clone(),len)){
                    candidates.push((res_path.clone(),len));
                }

                // восстанавливаем все удаленные элементы
                self.restore_removed(); 
            }        
            // берем самого оптимального кандидидата и доавлаем в ответ
            candidates.sort_by(|a,b|b.1.cmp(&a.1));
            if let Some(p) = candidates.pop() {
                result.push(p);
            } else {
                break;
            }
        }
        result
    }

}

use std::time::SystemTime;
fn main() {
    let mut g = Graph::new(5);
    g.add_edge(0,1,1);
    g.add_edge(0,2,4);
    g.add_edge(1,2,3);
    g.add_edge(1,3,2);
    g.add_edge(1,4,2);
    g.add_edge(3,2,5);
    g.add_edge(3,1,1);
    g.add_edge(4,3,3);
    let now = SystemTime::now();
    println!("{:?}",g.yens_alg(3,2,3,Alg::Ford));
    println!("{:?}",g.yens_alg(3,2,3,Alg::Dijkstra));
    println!("time: {:?}",now.elapsed().unwrap().as_millis());
}
