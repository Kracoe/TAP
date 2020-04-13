#include "tools.h"
#include "heap.h" // il faut aussi votre code pour heap.c


// Une fonction de type "heuristic" est une fonction h() qui renvoie
// une distance (double) entre une position de départ et une position
// de fin de la grille. La fonction pourrait aussi dépendre de la
// grille (comme par exemple du nombre de murs rencontrés par le
// segment départ-fin), mais on n'utilisera pas forcément ce dernier
// paramètre. Vous pouvez définir votre propre heuristique.
typedef double (*heuristic)(position,position,grid*);


// Heuristique "nulle" pour Dijkstra.
double h0(position s, position t, grid *G){
  return 0.0;
}


// Heuristique "vol d'oiseau" pour A*.
double hvo(position s, position t, grid *G){
  return fmax(abs(t.x-s.x),abs(t.y-s.y));
}


// Heuristique "alpha x vol d'oiseau" pour A*.
static double alpha=0; // 0 = h0, 1 = hvo, 2 = approximation ...
double halpha(position s, position t, grid *G) {
  return alpha*hvo(s,t,G);
}


// Structure "noeud" pour le tas min Q.
typedef struct node {
  position pos;        // position (.x,.y) d'un noeud u
  double cost;         // coût[u]
  double score;        // score[u] = coût[u] + h(u,end)
  struct node* parent; // parent[u] = pointeur vers le père, NULL pour start
} *node; // un arc dans un chemin

double distance(position pos){
  return abs()
}

node create_node(position pos_node, double cost, double score, node *parent){

  node a;
  a->pos = pos_node;
  a->cost = cost;
  a->score = score; 
  a->parent = parent;
  return a;
}


int comapre(const void *x, const void *y) {
  const double a = *(double*)x;
  const double b = *(double*)y;
  return (a<b)? -1 : (a>b); // ou encore return (a>b) - (a<b);
}


// Les arêtes, connectant les 8 cases voisines de la grille, sont
// valuées seulement par certaines valeurs. Le poids de l'arête u->v,
// noté w(u,v) dans le cours, entre deux cases u et v voisines est
// déterminé par la valeur de la case finale v. Plus précisément, si
// la case v de la grille contient la valeur C, le poids de u->v
// vaudra w(u,v) = weight[C] dont les valeurs numériques exactes sont
// définies ci-après. La liste des valeurs possibles d'une case est
// donnée dans "tools.h": V_FREE, V_WALL, V_WATER, ... Remarquer que
// weight[V_WALL]<0 ce qui n'est pas a priori une valuation correcte:
// A* marche seulement avec des poids positifs! Mais ce n'est pas un
// problème, puisqu'en position (i,j) si G.value[i][j] = V_WALL, alors
// c'est que le sommet à cette position n'existe pas! Aucune arête ne
// peut donc être incidente à (i,j).

double weight[]={
    1.0,  // V_FREE
  -99.9,  // V_WALL
    3.0,  // V_SAND
    9.0,  // V_WATER
    2.3,  // V_MUD
    1.5,  // V_GRASS
    0.1,  // V_TUNNEL
};


// Votre fonction A_star(G,h) doit construire un chemin dans la grille
// G entre la position G.start et G.end selon l'heuristique h(),
// évidemment selon l'algorithme A* vu en cours. Utilisez les notes de
// cours! S'il n'y a pas de chemin (par exemple si la destination est
// enfermée entre 4 murs), affichez un simple message d'erreur. Sinon,
// vous devez remplir le champs .mark de la grille pour que le chemin
// trouvé soit affiché plus tard par drawGrid(G). Il faut, par
// convention, avoir G.mark[i][j] = M_PATH ssi (i,j) appartient au
// chemin trouvé. Utilisez les touches a,z,+,-,p,c pour gérer la
// vitesse d'affichage et de progression de l'algorithme par
// exemple. Repportez-vous à "tools.h" pour avoir la liste des
// differentes touches et leurs actions, ainsi que les différentes
// valeurs possibles pour G.mark[][].
//
// Pour gérer l'ensemble P, servez-vous du champs G.mark[i][j] (=
// M_USED ssi (i,j) est dans P). Par défaut, ce champs est initialisé
// partout à M_NULL par initGrid().
//
// Pour gérer l'ensemble Q, vous devez utiliser un tas min de noeuds
// (type node) avec une fonction de comparaison (à créer) qui dépend
// du champs .score des noeuds. Pour la fonction de comparaison faites
// attention au fait que l'expression "(int)(8.1 - 8.2)" n'est pas
// négative, mais nulle! Vous devez utilisez la gestion paresseuse du
// tas (cf. le paragraphe du cours concernant l'implémentation de
// Dijkstra). Pensez qu'avec la gestion paresseuse du tas, la taille
// de Q est au plus la somme des degrés des sommets dans la grille.
// Pour visualiser un noeud de coordonnées (i,j) qui passe dans le tas
// Q vous pourrez mettre G.mark[i][j] = M_FRONT au moment où vous
// l'ajoutez.

void A_star(grid G, heuristic h){

  // Pensez à dessiner la grille avec drawGrid(G) à chaque fois que
  // possible, par exemple, lorsque vous ajoutez un sommet à P mais
  // aussi lorsque vous reconstruisez le chemin à la fin de la
  // fonction. Lorsqu'un sommet passe dans Q vous pourrez le marquer
  // M_FRONT (dans son champs .mark) pour le distinguer à l'affichage
  // des sommets de P (couleur différente).

  // Après avoir extrait un noeud de Q, il ne faut pas le détruire,
  // sous peine de ne plus pouvoir reconstruire le chemin trouvé! Vous
  // pouvez réfléchir à une solution simple pour libérer tous les
  // noeuds devenus inutiles à la fin de la fonction. Une fonction
  // createNode() peut simplifier votre code.

  // Les bords de la grille sont toujours constitués de murs (V_WALL) ce
  // qui évite d'avoir à tester la validité des indices des positions
  // (sentinelle).

  
  // Etape 1 
  // P est l'ensemble vide, on marque tous les sommets comme M_NULL


  int x0 = G->start->x;
  int y0 = G->start->y;
  
  G.mark[x0][y0] = M_FRONT;

  
  double s_score = h(G->start,G->end,G);
  node s = create_node(G->start,0,s_score,NULL);

  // On initialise Q, le tas
  
  int nmax = 8*(G->X)*(G->Y) ; //8*nb de cases
  heap Q = heap_create(nmax,h);
  heap_add(Q,s);
  

  while(Q != NULL){
    node u = heap_pop(Q); //On récupère le minimum de Q et on le supprime du tas
    if(u->pos == t->pos){ //On renvoie le chemin
      G.mark[u->pos->x][u->pos->y] = M_PATH;
      node a = u->parent;
      while(a->pos != s->pos){
        G.mark[a->pos->x][a->pos->y] = M_PATH;
        a = a->parent;
      }
      G.mark[s->pos->x][s->pos->y] = M_PATH ;
      return ;
    }
    // On ajoute u à P 
    G.mark[u->pos->x][u->pos->y] = M_USED;
    drawGrid();
    // Pour tout voisin de u qui n'est pas dans P
    for(int i = -1; i < 2; i++){
      for(int j = -1; j < 2;j++){
        if((i!=0)and(j!=0)){  // Ou si on est sur un mur
          double c = u->cost + weight[G.value[i][j]];
          if(G.mark[i][j] != M_USED ){
            G.mark[i][j] = M_FRONT;
            position v_pos;
            u_pos->x = i;
            u_pos->y = j;
            double v_score = c + h(v_pos,G->end,G);
            node v_node = create_node(v_pos,c,v_score,u);
            heap_add(Q,v_node);
          }
        }
      }
    }
  }
  return("Le chemin n'a pas été trouvé");



  // Améliorations à faire seulement quand vous aurez bien avancé:
  //
  // (1) Une fois la cible atteinte, afficher son coût ainsi que le
  // nombre de sommets visités (somme des .mark != M_NULL). Cela
  // permettra de comparer facilement les différences d'heuristiques,
  // h0() vs. hvo().
  //
  // (2) Le chemin a tendance à zizaguer, c'est-à-dire à utiliser
  // aussi bien des arêtes horizontales que diagonales (qui ont le
  // même coût), même pour des chemins en ligne droite. Essayer de
  // rectifier ce problème d'esthétique en modifiant le calcul de
  // score[v] de sorte qu'à coût[v] égale les arêtes (u,v)
  // horizontales ou verticales soient favorisées. Bien sûr, votre
  // modification ne doit en rien changer la distance entre .start et
  // .end.
  //
  // (3) Modifier votre implémentation du tas dans heap.c de façon à
  // utiliser un tableau de taille variable, en utilisant realloc() et
  // une stratégie "doublante": lorsqu'il n'y a pas plus assez de
  // place dans le tableau, on double sa taille avec un realloc(). On
  // peut imaginer que l'ancien paramètre 'nmax' devienne non pas le
  // nombre maximal d'éléments, mais sa taille maximale initiale
  // (comme par exemple nmax=4).
  //
  // (4) Gérer plus efficacement la mémoire en libérant les noeuds
  // devenus inutiles. Pour cela, il faut lire la note ci-après.
  //

  ;;;

  // Note pour libérer les noeuds utilisée dans A_star(). À faire
  // seulement si vous avez déjà bien avancé.
  //
  // On ajoute un champs .nchild à la structure node, permettant de
  // gérer le nombre de fils qu'un node de P ou Q possède. C'est
  // relativement léger à gérer puisqu'on augmente .nchild de u chaque
  // fois qu'on fait parent[v]=p, soit juste après "node v =
  // createNode(p,...)". Pensez à faire .nchild=0 dans createNode().
  // Notez bien qu'ici on parle de "node", donc des copies de sommets.
  //
  // L'observation utile est que tous les nodes de Q sont des
  // feuilles. On va alors pouvoir se débarrasser de tous les nodes
  // ancêtres de ces feuilles simplement on extrayant les nodes de Q
  // dans n'importe quel ordre. (Si on veut être plus efficace que
  // |Q|*log|Q|, on peut vider le tableau .array[] directement sans
  // passer par heap_pop(). Pour être propre il faudrait peut-être
  // ajouter une fonctions comme "void* heap_get(int i)" qui
  // permettrait d'extraire l'objet numéro i sans modifier le tas, et
  // renvoie NULL s'il est vide). On supprime alors chaque node, en
  // mettant à jour le nombre de fils de son père, puis en supprimant
  // le père s'il devient feuille (son .nchild passe 0) et ainsi de
  // suite. On élimine ainsi l'arbre par branches qui se terminent
  // toutes dans Q.
  //
  // Ce processus peut ne pas fonctionner si P contient des nodes qui
  // sont des feuilles. L'observation est alors que de tels nodes ne
  // peuvent pas être sur le chemin s->t. On peut donc les supprimer
  // au fur et à mesure directement dans la boucle principale quand on
  // les détecte. On voit qu'un tel node apparaît si après avoir
  // parcouru tous les voisins de u aucun node v n'est créé (et ajouté
  // dans Q). Il suffit donc de savoir si on est passé par heap_add()
  // (ou encore de comparer la taille de Q avant et après la boucle
  // sur les voisins). Si u est une feuille, on peut alors supprimer
  // le node u, mettre à jour .nchild de son père et remonter la
  // branche jusqu'à trouvé un node qui n'est plus une feuille. C'est
  // donc la même procédure d'élagage que précdemment qu'on pourrait
  // capturer par une fonction freeNode(node p).
}

int main(int argc, char *argv[]){

  unsigned seed=time(NULL)%1000;
  printf("seed: %u\n",seed); // pour rejouer la même grille au cas où
  srandom(seed);

  // testez différentes grilles ...

  grid G = initGridPoints(80,60,V_FREE,1); // petite grille vide, sans mur
  //grid G = initGridPoints(width,height,V_FREE,1); // grande grille vide, sans mur
  //grid G = initGridPoints(32,24,V_WALL,0.2); // petite grille avec quelques murs
  //grid G = initGridLaby(64,48,4); // petit labyrinthe aléatoire
  //grid G = initGridLaby(width/8,height/8,3); // grand labyrinthe aléatoire
  //grid G = initGridFile("mygrid.txt"); // grille à partir d'un fichier

  // ajoutez à G une (ou plus) "région" de texture donnée ...
  // (déconseillé pour initGridLaby() et initGridFile())

  //addRandomBlob(G, V_WALL,   (G.X+G.Y)/20);
  //addRandomBlob(G, V_SAND,   (G.X+G.Y)/15);
  //addRandomBlob(G, V_WATER,  (G.X+G.Y)/6);
  //addRandomBlob(G, V_MUD,    (G.X+G.Y)/3);
  //addRandomBlob(G, V_GRASS,  (G.X+G.Y)/15);
  //addRandomBlob(G, V_TUNNEL, (G.X+G.Y)/4);

  // sélectionnez des positions s->t ...
  // (inutile pour initGridLaby() et initGridFile())

  G.start=(position){0.1*G.X,0.2*G.Y}, G.end=(position){0.8*G.X,0.9*G.Y};
  //G.start=randomPosition(G,V_FREE); G.end=randomPosition(G,V_FREE);

  // constantes à initialiser avant init_SDL_OpenGL()
  scale = fmin((double)width/G.X,(double)height/G.Y); // zoom courant
  hover = false; // interdire les déplacements de points
  init_SDL_OpenGL(); // à mettre avant le 1er "draw"
  drawGrid(G); // dessin de la grille avant l'algo
  update = false; // accélère les dessins répétitifs

  alpha=0;
  A_star(G,halpha); // heuristique: h0, hvo, alpha*hvo

  update = true; // force l'affichage de chaque dessin
  while (running) { // affiche le résultat et attend
    drawGrid(G); // dessine la grille
    handleEvent(true); // attend un évènement
  }

  freeGrid(G);
  cleaning_SDL_OpenGL();
  return 0;
}
