use macroquad::prelude::*;
use ::rand::Rng;
use ::rand::rngs::ThreadRng;

// İki boyutlu bir noktayı temsil eden yapı
#[derive(Clone, Copy)]
struct Point {
    x: f32,
    y: f32,
}

impl Point {
    // İki nokta arasındaki öklid mesafesini hesaplayan fonksiyon
    fn distance(&self, other: &Point) -> f32 {
        ((self.x - other.x).powi(2) + (self.y - other.y).powi(2)).sqrt()
    }
}

// Düğüm yapısı: bir nokta, ebeveyn düğüm indeksi ve maliyet içerir
#[derive(Clone)]
struct Node {
    point: Point,
    parent: Option<usize>,
    cost: f32,
}

impl Node {
    // Yeni bir düğüm oluşturan yardımcı fonksiyon
    fn new(point: Point, parent: Option<usize>, cost: f32) -> Self {
        Node { point, parent, cost }
    }
}

// RRT* algoritmasını tanımlayan yapı
struct RRTStar {
    nodes: Vec<Node>, // Ağaçtaki düğümler
    goal: Point,    // Hedef nokta
    step_size: f32, // Adım boyutu
    goal_threshold: f32, // Hedef eşiği
    search_radius: f32, // Yakınlık yarıçapı
    best_cost: f32,     // En iyi maliyet
    rng: ThreadRng,     // Rastgele sayı üreteci
}

impl RRTStar {
    // RRT* algoritmasını başlatan fonksiyon
    fn new(start: Point, goal: Point, step_size: f32, goal_threshold: f32, search_radius: f32) -> Self {
        // Başlangıç düğümünü kök olarak ekler, maliyet sıfırdır
        let root = Node::new(start, None, 0.0);
        RRTStar {
            nodes: vec![root],
            goal,
            step_size,
            goal_threshold,
            search_radius,
            best_cost: f32::INFINITY, // Başlangıçta en iyi yol maliyeti sonsuz
            rng: ::rand::thread_rng(),
        }
    }

    // Rastgele bir nokta oluşturan fonksiyon
    fn random_point(&mut self, min_x: f32, max_x: f32, min_y: f32, max_y: f32) -> Point {
        let x = self.rng.gen_range(min_x..max_x);
        let y = self.rng.gen_range(min_y..max_y);
        Point { x, y }
    }

        // Verilen bir noktaya en yakın düğümün indeksini bulur
    fn find_nearest(&self, point: &Point) -> usize {
        self.nodes
            .iter()
            .enumerate()
            .min_by(|(_, a), (_, b)| {
                a.point
                    .distance(point)
                    .partial_cmp(&b.point.distance(point))
                    .unwrap()
            })
            .map(|(index, _)| index)
            .unwrap()
    }

    // Bir noktadan diğerine belirli bir açıyla ilerleyen fonksiyon
    fn steer(&self, from: &Point, to: &Point) -> Point {
        let angle = (to.y - from.y).atan2(to.x - from.x);
        Point {
            x: from.x + self.step_size * angle.cos(),
            y: from.y + self.step_size * angle.sin(),
        }
    }

    // Çarpışma kontrol fonksiyonu ??????
    fn is_collision_free(&self, _point: &Point) -> bool {
        true
    }

    // Yeni bir düğüm ekler ve bu düğüm için maliyet hesaplar
    fn add_node(&mut self, point: Point, parent_index: usize) -> usize {
        let cost = self.nodes[parent_index].cost + point.distance(&self.nodes[parent_index].point);
        let new_node = Node::new(point, Some(parent_index), cost);
        self.nodes.push(new_node);
        self.nodes.len() - 1
    }

    // Yeni eklenen düğümün yakınında olan düğümlerin indekslerini döndürür
    fn near(&self, new_node_index: usize) -> Vec<usize> {
        let new_node = &self.nodes[new_node_index];
        self.nodes
            .iter()
            .enumerate()
            .filter(|(i, node)| *i != new_node_index && node.point.distance(&new_node.point) < self.search_radius)
            .map(|(i, _)| i)
            .collect()
    }

    // Daha kısa maliyetli yollar bulunursa düğümleri yeniden bağlar
    fn rewire(&mut self, new_node_index: usize) {
        let neighbors = self.near(new_node_index);
        let new_node = self.nodes[new_node_index].clone();

        for &neighbor_index in &neighbors {
            let neighbor = &self.nodes[neighbor_index];
            let new_cost = new_node.cost + new_node.point.distance(&neighbor.point);

            // Eğer yeni maliyet mevcut maliyetten düşükse, düğümü yeniden bağla
            if new_cost < neighbor.cost {
                self.nodes[neighbor_index].parent = Some(new_node_index);
                self.nodes[neighbor_index].cost = new_cost;
            }
        }
    }

    // En iyi yolu günceller, eğer hedefe ulaşılmış ve maliyet iyileşmişse 'true' döner
    fn update_best_path(&mut self) -> bool {
        let last_node = &self.nodes[self.nodes.len() - 1];
        let distance_to_goal = last_node.point.distance(&self.goal);
        
        if distance_to_goal < self.goal_threshold && last_node.cost < self.best_cost {
            self.best_cost = last_node.cost;
            return true;
        }
        false
    }

    // En iyi yolu geri izleyerek bir noktalar dizisi döner
    fn trace_path(&self) -> Vec<Point> {
        let mut path = Vec::new();
        let mut current_node_index = self.nodes.len() - 1;

        while let Some(parent_index) = self.nodes[current_node_index].parent {
            path.push(self.nodes[current_node_index].point);
            current_node_index = parent_index;
        }
        path.push(self.nodes[current_node_index].point);
        path.reverse();
        path
    }
}

#[macroquad::main("RRT* Visualization")]
async fn main() {
    let mut rng = ::rand::thread_rng();
    
    // Rastgele bir başlangıç ve hedef noktası seçilir
    let start = Point {
        x: rng.gen_range(0.0..400.0),
        y: rng.gen_range(0.0..400.0),
    };
    let goal = Point {
        x: rng.gen_range(0.0..400.0),
        y: rng.gen_range(0.0..400.0),
    };
    
    // RRT* ağacı başlatılır
    let mut rrt_star = RRTStar::new(start, goal, 10.0, 10.0, 15.0);
    let mut optimal_path: Vec<Point> = Vec::new();
    let mut iterations = 0;
    let max_iterations = 5000; // İstenilen iterasyon sınırı

    loop {
        if iterations >= max_iterations {
            println!("Optimal path found within iteration limit.");
            break;
        }

        // Rastgele bir nokta oluştur ve ağaca en yakın düğümü bul
        let rand_point = rrt_star.random_point(0.0, 400.0, 0.0, 400.0);
        let nearest_index = rrt_star.find_nearest(&rand_point);
        let nearest_node = &rrt_star.nodes[nearest_index];
        let new_point = rrt_star.steer(&nearest_node.point, &rand_point);

        // Çarpışma kontrolünden geçerse düğümü ekle ve yeniden bağla
        if rrt_star.is_collision_free(&new_point) {
            let new_node_index = rrt_star.add_node(new_point, nearest_index);
            rrt_star.rewire(new_node_index);

            // Yeni bir en iyi yol bulunursa optimal yolu güncelle
            if rrt_star.update_best_path() {
                optimal_path = rrt_star.trace_path();
                println!("New optimal path with cost: {}", rrt_star.best_cost);
            }
        }

        clear_background(WHITE);

        // Düğümler arası bağlantıları çiz
        for node in &rrt_star.nodes {
            if let Some(parent_index) = node.parent {
                let parent_node = &rrt_star.nodes[parent_index];
                draw_line(
                    node.point.x,
                    node.point.y,
                    parent_node.point.x,
                    parent_node.point.y,
                    1.0,
                    BLUE,
                );
            }
        }

        // Optimal yol (eğer bulunmuşsa) yeşil çizgi ile gösterilir
        for i in 1..optimal_path.len() {
            let start = optimal_path[i - 1];
            let end = optimal_path[i];
            draw_line(start.x, start.y, end.x, end.y, 2.0, GREEN);
        }

        // Başlangıç ve hedef noktalarını daire olarak çiz
        draw_circle(rrt_star.nodes[0].point.x, rrt_star.nodes[0].point.y, 5.0, GREEN);
        draw_circle(rrt_star.goal.x, rrt_star.goal.y, 5.0, RED);

        next_frame().await; // Ekranı güncelle
        iterations += 1;    // İterasyon sayacını artır
    }
}
