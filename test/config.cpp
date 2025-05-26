// // Parametre config[] = {
// //   {"Pas en mm pour moteur X", 10},
// //   {"Pas en mm pour moteur y", 5},
// //   {"Pas en mm pour moteur z", 2},
// //   {"Nombre de lignes A", 10},
// //   {"Nombre de colonnes A", 10},
// //   {"Nombre de lignes B", 2},
// //   {"Nombre de colonnes B", 3},
// //   {"Pas entre deux positions sur l'axe X pour la matrice A", 5},
// //   {"Pas entre deux positions sur l'axe Y pour la matrice A", 2},
// //   {"Pas entre deux positions sur l'axe X pour la matrice B", 5},
// //   {"Pas entre deux positions sur l'axe Y pour la matrice B", 1},
// //   {"Position initiale sur l'axe X pour la matrice A", 50},
// //   {"Position initiale sur l'axe Y pour la matrice A", 10},
// //   {"Position initiale sur l'axe X pour la matrice B", 150},
// //   {"Position initiale sur l'axe Y pour la matrice B", 20},
// //   {"test", 0}
// // };

// //nbre de pas par mm pour chaque moteur (prévoir de les acquerrer depuis l'interface web)
// int nbre_pas_par_mm_pour_stepperX=config[0].valeur;
// int nbre_pas_par_mm_pour_stepperY=config[1].valeur;
// int nbre_pas_par_mm_pour_stepperZ=config[2].valeur;

// //dimension de la matrice A
// int MA=config[3].valeur;
// int NA= config[4].valeur;    
// std::vector<std::vector<int>> matriceA(MA, std::vector<int>(NA));

// //dimension de la matrice B
// int MB=config[5].valeur ;
// int NB= config[6].valeur;
// std::vector<std::vector<int>> matriceB(MB, std::vector<int>(NB));

// //les pas entre deux positions pour A et B en mm (prévoir de les calculer en pas) (prévoir de les acquérir depuis l'interface web)
// int pasAX=config[7].valeur;
// int pasAY=config[8].valeur;
// int pasBX=config[9].valeur;
// int pasBY=  config[10].valeur;

// //positions initiales pour les deux matrices en mm(prévoir de les calculer en pas) (prévoir de les acquérir depuis l'interface web)
// int posAX = config[11].valeur;
// int posAY= config[12].valeur;
// int posBX  = config[13].valeur;
// int posBY = config[14].valeur;
