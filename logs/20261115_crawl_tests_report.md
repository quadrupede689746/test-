# Essais levée des pieds et crawl — 15 novembre 2026

## Configuration

- Contrôleur position, 1000 Hz, `kp=20`, `ki=0`, `kd=0.1` sur les 12 articulations; limite d'effort 4 Nm.
- Pose initiale fléchie : chaque jambe `(hip=0, upper=-0.68162889, lower=1.26606748)` rad.
- Monde Gazebo `crawl.world`; masses jambe/pied conservées à 0.10/0.03 kg.
- Compilation `colcon build --packages-select open_quadruped --symlink-install` réussie.

## Tableau variable | résultat | conclusion

| Variable / essai | Résultat mesuré | Conclusion |
|---|---|---|
| Bumper, sphère pied rayon 20 mm, `minDepth=1 mm` | Le capteur indique les collisions réelles pied/sol; au-delà d'environ 1–2 cm de levée, le contact s'efface. | Le bumper n'est pas bloqué. Une consigne de 1 cm est proche du rayon/compression/contact et trop faible pour conclure à une levée nette. |
| Commande verticale 3 cm | Les essais initiaux ont donné 13.5–21.9 mm de levée monde selon la patte; le contact se libère. | 3 cm de consigne n'assure pas 3 cm de déplacement monde. |
| Commande verticale 5 cm, transfert de charge | FL 39.4 mm, FR 39.4 mm, BL 34.4 mm, BR 34.3 mm de levée monde maximale. Les trois appuis restants restent en contact. Marge COM dynamique minimale : FL 22.7 mm, FR 23.0 mm, BL 25.1 mm, BR 25.2 mm. | Les quatre pattes passent le critère de levée nette >=3 cm et marge >2 cm. La consigne 5 cm n'est pas entièrement atteinte, principalement à cause du déplacement du corps et de la compliance. Gains inchangés. |
| Premier crawl, marge planifiée 25 mm | Marge reconstruite avec pieds/COM mesurés : seulement 8.3–8.6 mm aux phases avant. Le robot a levé les pieds, mais l'effort a atteint 4 Nm et l'erreur max de suivi a été 0.128 rad. | La marge calculée à partir des seules cibles surestimait la marge réelle. Ce premier essai n'est pas validé. CSV `20261115_123538_crawl_tiny_step_5mm_final.csv`. |
| Crawl avec transfert cible 40 mm et verrou mesuré 22 mm | Environ 4.8 cycles sur 116.2 s simulées. Une seule patte perd le contact à la fois. Hauteur monde reconstruite par FK : FL 39–46 mm, FR 44–45 mm, BL 49–51 mm, BR 47–52 mm. Marges COM mesurées minimales : FL 27.2 mm, FR 27.5 mm, BL 44.1 mm, BR 39.7 mm. | Levée alternée et transfert de charge mesurable, marge >2 cm pendant les phases sans appui. Crawl expérimental obtenu. |
| Stabilité / suivi du crawl | Corps z = 0.181–0.196 m; |roll| max 0.032 rad; |pitch| max 0.071 rad; erreur articulaire max 0.107 rad. Effort exactement à 4 Nm pendant 26 échantillons sur 5820. | Pas de chute pendant l'essai, mais il reste des saturations de couple et une erreur de suivi mesurable. kp20/kd0.1 suffisent pour produire les levées; ne pas augmenter les gains sans essai dédié, car la limite d'effort est déjà atteinte. |
| Déplacement du corps pendant le crawl | x : -0.379 m; y : +0.014 m dans le repère monde, alors que le pas demandé était 5 mm. | Le robot a avancé/derivé de façon cyclique, mais la distance parcourue n'est pas cohérente avec les pas de 5 mm; le contrôle de déplacement reste à calibrer. Ne pas présenter ce résultat comme une marche métriquement validée. |

## Modifications de contrôle

- `keyboard_walk.py` utilise la pose fléchie mesurée, les gains de simulation existants, une compensation XY de la trajectoire de swing, un transfert de poids, et refuse le décollage si un des trois pieds porteurs perd le contact.
- Les pieds d'appui et le COM sont maintenant évalués à partir des états articulaires mesurés avant la levée. `support_margin_m` (cible de transfert) vaut 0.04 m et `minimum_support_margin_m` (verrou de sécurité) vaut 0.022 m.
- `crawl_sim.launch.py` démarre sur la pose fléchie, à z=0.245 m avant le placement contrôlé des quatre pieds.
- Le logger marque toute perte de contact comme anomalie, y compris la perte prévue du pied qui swingue. Pour analyser une marche, comparer les contacts des trois pieds de support au pied en swing.

## Logs

- Levées 5 cm avec marge COM dynamique : `20261115_122103_foot_lift_front_left_0.050m.csv`, `20261115_122155_foot_lift_front_right_0.050m.csv`, `20261115_122317_foot_lift_back_left_0.050m.csv`, `20261115_122413_foot_lift_back_right_0.050m.csv`.
- Crawl initial (marge insuffisante) : `20261115_123538_crawl_tiny_step_5mm_final.csv` et son résumé.
- Crawl avec marge reconstruite : `20261115_124358_crawl_margin_40mm_min22.csv` et son résumé/paramètres.

Aucun push GitHub effectué.
