open Raylib

let width = 1000
let height = 1000

type vec = float * float

type boid = 
  {
    position : vec;
    speed : vec;
    color : float;
  }

type tfac =
  { 
    cohesion : float;
    alignement : float;
    separation : float;
  }

type args = {
  boids : boid list;
  facteurs : tfac;
}

let seuil = 1e4 (* distance pour observer les autres boids *)
let speedlimit = 150.0 (* vitesse max des boids*)
let dt = 1.0/.60.0

let zero = 0.0,0.0
let dist (x1,y1) (x2,y2) = (x1 -. x2)**2.0 +. (y1 -. y2)**2.0
let norme_squared = dist zero
let norme a = a |> norme_squared |> sqrt

let (+$) (x1,y1) (x2,y2) = x1+.x2, y1+.y2
let (-$) (x1,y1) (x2,y2) = x1-.x2, y1-.y2
let ( *$) (x,y) q = x*.q,y*.q
let (/$) (x,y) q = x/.q,y/.q

let vect_elem a b =
  let v = b -$ a in
  if dist a b  < 0.1 then zero else
  v /$ norme v

let get_angle (x,y) = assert ( norme_squared (x,y) > 0.0) ;  atan2 y x
let from_angle theta = (cos theta, sin theta)

let acc pos (ar,n) fac = 
  let separation = 
    (List.fold_left (+$) zero (List.map (fun x -> vect_elem x.position pos) ar)) *$ fac.separation
  in
  let alignement =
    if n = 1.0 then zero else
    let midangle = List.fold_left (+.) 0.0 (List.map (fun x -> if x.position = pos then 0.0 else get_angle x.speed) ar) /. (n-.1.0) in
    from_angle midangle *$ fac.alignement
  in
  let cohesion =
    let midpos = List.fold_left (+$) zero (List.map (fun x-> x.position) ar) /$ n in
    vect_elem pos midpos *$ fac.cohesion
  in
  let acceleration =  cohesion  +$  separation +$ alignement
 in
  assert (Float.is_finite (fst acceleration));
  assert (Float.is_finite (snd acceleration));

  vect_elem zero acceleration


let around pos = 
  List.filter (fun x -> let d = dist x.position pos in d < seuil) 

let update_boid args {position;speed;color} = 
  (*euler*)
    let autour = around position args.boids in
    let n = autour |> List.length |> float_of_int in
    let vx,vy = speed +$ acc position (autour,n) args.facteurs in
    let x,y = position in
    let speed' = 
      (
        (if x < 0.0 || x > float_of_int width then -.vx else vx),
        if y < 0.0 || y > float_of_int height then -.vy else vy
     ) in 
    let speed' = if norme speed' > speedlimit then vect_elem zero speed' *$ speedlimit else speed' in
    {
      position = position +$ (speed' *$ dt);
      speed = speed'; 
      color = 0.99 *. color +. 0.01 *. List.fold_left (+.) 0.0 (List.map (fun x-> x.color) autour) /.n  +. Random.float 10.0 -. 5.0 
    }

let update args =
  {args with boids = List.map (update_boid args) args.boids}

let demipi = 3.14 /. 2.
let vec_from_angle (x,y) q alpha =  Vector2.create (x+. q*.cos alpha) (y+.q*.sin alpha) 

let draw_boid others {position;speed;color} =
  let theta = get_angle speed  in
  let top = vec_from_angle position 20.0 theta in
  let g = vec_from_angle position 5.0 (theta+.demipi) in
  let d = vec_from_angle position 5.0 (theta-.demipi) in
  draw_triangle top d g (color_from_hsv color 0.5 1.0)
  (*let x',y' = position+$speed in
  draw_line (int_of_float x) (int_of_float y) (int_of_float x') (int_of_float y') Color.yellow;
  List.iter (fun p ->
    let x',y' = p.position in
    draw_line (int_of_float x) (int_of_float y) (int_of_float x') (int_of_float y') Color.white) (around position others)
  *)


let draw {boids;facteurs} = 
  begin_drawing ();
  (*
  draw_rectangle 0 0 width height (fade Color.black 1.0);
  draw_rectangle 0 height width 20 Color.black;*)
  clear_background Color.black;
  draw_text 
    (Printf.sprintf "cohesion (A/Z) : %.0f | alignement (Q/S) : %.0f | separation (W/X) : %.0f | space for more boids" facteurs.cohesion facteurs.alignement facteurs.separation)
  0 height 20 Color.white;
  List.iter (draw_boid boids) boids;
  end_drawing ()


let rec loop args = 
  if window_should_close () then close_window () else begin
    draw args;
    let boids' = if is_key_down Key.Space then ({position = (float_of_int width, float_of_int height) /$ 2.0;speed=Random.float 10.0 -. 5.0,Random.float 10.0 -. 5.0; color = Random.float 360.0 }::args.boids) else args.boids in
    let agrand = 10.0 in
    let fac' = {
      cohesion = args.facteurs.cohesion +. agrand *. ((if is_key_pressed Key.Q then 1.0 else 0.0) +. (if is_key_pressed Key.W then -1.0 else 0.0));
      alignement = args.facteurs.alignement +. agrand *. ((if is_key_pressed Key.A then 1.0 else 0.0) +. (if is_key_pressed Key.S then -1.0 else 0.0));
      separation = args.facteurs.separation +. agrand *. ((if is_key_pressed Key.Z then 1.0 else 0.0) +. (if is_key_pressed Key.X then -1.0 else 0.0));
    }
    in
    {facteurs = fac'; boids = boids'} |> update |> loop
  end

let () = 
  init_window width (height+20) "boids goes brrrr";
  Random.self_init ();
  set_target_fps 60;
  let fac =
  {
    cohesion = 180.0;
    alignement = 80.0;
    separation = 10.0;
  } in
  let boids = List.init 50 (fun _ -> 
    {
      position = Random.float (float_of_int width), Random.float (float_of_int height);
      speed = ((Random.float 10.0 , Random.float 10.0) +$ (-5.0, -5.0)) *$ 1.0;
      color = Random.float 360.0
    })
 in
 loop {boids = boids; facteurs = fac}
