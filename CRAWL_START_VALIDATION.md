# Validation du démarrage du crawl — 15 novembre 2026

## Résumé

Le démarrage du crawl n’est pas encore reproductiblement franchi : le clavier attend bien une pose stable, mais le robot n’atteint pas la marge de support minimale avant le premier lever de FL. La répétition de la pose debout est très reproductible; c’est le transfert de charge pendant la commande qui bloque.

Les essais à 8 et 12 Nm sont restés temporaires. Le Xacro a été restauré à 4 Nm après ces essais, et la pose de départ ainsi que les paramètres de marche ont été remis à leurs valeurs nominales. Les STL, les gains et la configuration finale d’effort n’ont pas été modifiés.

## 1. Répétabilité de la pose debout

Chaque essai est un nouveau lancement de `crawl_sim.launch.py`, logger lancé directement comme processus contrôlé, fenêtre stable `t=30–60 s` simulées. Les marges sont calculées à chaque échantillon depuis les angles mesurés, les FK et le COM dérivés de l’URDF, en retirant virtuellement une patte du polygone de support. Aucun pied n’est réellement levé pendant ces essais.

| Lancement | Marge sans FL | sans FR | sans BL | sans BR | Contacts FL/FR/BL/BR | Effort absolu max des 12 joints |
|---|---:|---:|---:|---:|---|---:|
| 1 | −16,09 mm | −15,93 mm | +15,93 mm | +16,09 mm | 100/100/100/100 % | 0,61 Nm |
| 2 | −16,09 mm | −15,93 mm | +15,93 mm | +16,09 mm | 100/100/100/100 % | 0,61 Nm |
| 3 | −16,09 mm | −15,93 mm | +15,93 mm | +16,09 mm | 100/100/100/100 % | 0,61 Nm |
| 4 | −16,09 mm | −15,92 mm | +15,92 mm | +16,09 mm | 100/100/100/100 % | 0,61 Nm |
| 5 | −16,09 mm | −15,92 mm | +15,92 mm | +16,09 mm | 100/100/100/100 % | 0,61 Nm |

Pic d’effort par articulation sur les cinq fenêtres (Nm, maximum absolu) :

| Patte | Hip | Upper-leg | Lower-leg |
|---|---:|---:|---:|
| FL | 0,10 | 0,13 | 0,61 |
| FR | 0,13 | 0,15 | 0,61 |
| BL | 0,10 | 0,13 | 0,41 |
| BR | 0,11 | 0,14 | 0,40 |

Le COM relatif au centroïde des quatre pieds mesuré dans ces fenêtres est `(+23,73 à +23,74 mm, +0,09 à +0,10 mm)` : il est décalé vers l’avant et pratiquement centré latéralement. La hauteur varie de 0,025 à 0,033 mm par fenêtre; x varie de 0,043 à 0,052 mm. Contacts et efforts à l’arrêt sont stables. Les cinq journaux propres sont `logs/20261115_154233_repro_clean_1.csv` à `logs/20261115_154717_repro_clean_5.csv`; les résumés et paramètres correspondants sont dans les fichiers portant les mêmes préfixes.

**Pourquoi 27–44 mm avant, contre −5,4 mm maintenant ?** Ce ne sont pas deux mesures de la même phase. Les 27,2–44,1 mm du rapport de crawl précédent ont été relevés pendant les cycles où le transfert de poids avait déjà eu lieu avec une cible de 40 mm. Le −5,4 mm récent est mesuré avant le premier lever FL, alors que le contrôleur actuel réduit le transfert pour cause d’IK et finit par revenir à zéro. Les cinq répétitions statiques confirment une pose identique; elles n’expliquent pas le delta par de la variabilité entre lancements. Le calcul URDF statique donne une marge contrefactuelle plus basse encore (environ −13 mm devant) que les marges mesurées en simulation (environ −16 mm), principalement parce que le robot en simulation n’est pas exactement à la pose articulaire nominale et que la marge est sensible à quelques millimètres de position des pieds/COM.

La première série de cinq journaux (`logs/20261115_151752_repro_1.csv` à `logs/20261115_152308_repro_5.csv`) avait plusieurs processus logger concurrents; elle n’est pas utilisée pour les résultats ci-dessus. Elle reste dans le workspace pour traçabilité. Les cinq fichiers `repro_clean` proviennent de démarrages distincts avec un seul logger actif à la fois.

## 2. COM URDF et géométrie

Le calcul FK/inertiel du Xacro développé donne une masse totale de **3,14 kg**. Dans la pose fléchie nominale :

- COM dans `base_link` : `(-6,46 mm, 0 mm, -23,45 mm)`;
- centroïde du rectangle des pieds : `(-26,00 mm, 0 mm, -180,00 mm)`;
- le COM est donc **19,54 mm en avant** du centre des pieds, avec moins de 0,01 mm de décalage latéral dans le modèle symétrique;
- si l’on retire virtuellement une patte avant, la marge du modèle est −13,06 mm; sans une patte arrière, +13,06 mm.

La batterie ne pèse que 0,40 kg. La déplacer de 50 mm vers l’arrière ne déplacerait le COM total que d’environ 6,4 mm; cela peut aider, mais ne suffit pas seul à créer 30 mm de marge. Je n’ai pas modifié sa position.

## 3. Essais de couple et de transfert

| Variable | Résultat mesuré | Conclusion |
|---|---|---|
| Limite nominale 4 Nm, commande `w` | BL upper-leg atteint 4,00 Nm; verrou FL à −5,4 mm; arrêt avant le lever | Saturation observée; crawl bloqué. Journal `20261115_151237_trial.csv`. |
| Limite d’essai 8 Nm | BL upper-leg max 1,21 Nm, aucune saturation à 8; marge au verrou +0,8 mm | La saturation BL disparaît dans cet essai, mais le seuil +22 mm n’est toujours pas atteint. Aucun cycle. Journal `20261115_152703_effort_8.csv`. |
| Limite d’essai 12 Nm | BL upper-leg max 0,712 Nm, aucune saturation à 12; marge au verrou −7,2 mm | Pas d’amélioration de marge; aucun cycle. Le couple seul n’explique pas le blocage. Journal `20261115_152848_effort_12.csv`. |
| Pose pieds élargis de 20 mm par patte (avant/arrière et latéral), 4 Nm | Marge avant FL +10,9 mm | Amélioration par rapport à −5,4 mm, mais encore sous +22 mm. Journal `20261115_153145_wide_stance.csv`. |
| Pose pieds élargis de 30 mm, 4 Nm | Marge avant FL +13,5 mm | Encore sous le seuil; pas de lever. Journal `20261115_153348_wide30.csv`. |
| Cible de transfert augmentée seule à 45 mm, pose nominale | Transfert réduit à 50 puis 25 % pour garder les cibles dans les limites; marge +11,0 mm | Hausser la cible seule ne débloque pas l’IK. Journal `20261115_153513_margin45.csv`. |
| Pose élargie de 30 mm et cible 45 mm | Aucune cible IK complète, BR résidu 2,8 mm | La combinaison échoue aussi; pas de lever. Journal `20261115_154943_wide30_margin45.csv`. |

Pour les essais 8/12 Nm, les limites `effort` des 12 articulations ont été temporairement changées dans le Xacro. Elles ont ensuite été restaurées et vérifiées à 4,0 Nm.

### Candidat IK, non validé dans Gazebo

En déplaçant chaque cible de pied de 30 mm vers l’extérieur (avant +x, arrière −x, pieds gauche +y, droite −y), l’IK trouve cette pose fléchie :

```text
FL  (+0.1442, -0.7160, 1.0609) rad
FR  (-0.1442, -0.7160, 1.0609) rad
BL  (+0.1443, -0.4113, 1.0362) rad
BR  (-0.1443, -0.4113, 1.0362) rad
```

Avec un transfert de pieds calculé pour viser 45 mm avant chaque levée, le modèle IK prévoit des marges finales de 37,5 mm (levée avant) et 39,6 mm (levée arrière). Ce calcul statique est atteignable par IK dans la pose nominale du solveur, mais le test Gazebo world-anchor a échoué sur BR à 2,8 mm de résidu. Ce n’est donc pas une pose de marche validée. Écarter davantage (40 mm par patte) faisait déjà échouer les transferts avant sur les tests IK hors Gazebo.

## 4. Récupération par `r`

J’ai provoqué un roulis de 1,0 rad via `/gazebo/set_entity_state`. Le safety monitor a déclaré FALL et le logger a enregistré le basculement. La touche `r` a retourné `success=True`; après 10 s, le logger mesurait quatre contacts, z=0,1899 m, roll=0,00024 rad et pitch=0,02783 rad. Journal `logs/20261115_153639_reset_fall.csv`.

Cet essai a révélé et corrigé un vrai défaut : le code utilisait `/gazebo/set_model_state`, qui n’a pas de serveur actif dans cette installation Humble/Gazebo Classic. Il utilise désormais le service actif `/gazebo/set_entity_state` et vérifie la réponse.

## 5. Suite des tests

T2–T10 ne sont pas lancés : les essais précédents n’ont pas réussi à terminer le premier transfert ni à lever une patte avec une marge suffisante. Distance, signe `w`, latéral, rotation, arrêt en phase, perturbation/récupération pour un vrai cycle, accélération, et parcours d’un mètre restent **non validés**.

Vérifications finales : build `colcon` réussi; pytest `5 passed`; aucun processus `gzserver`/`gzclient` laissé actif. Aucun push GitHub effectué.
