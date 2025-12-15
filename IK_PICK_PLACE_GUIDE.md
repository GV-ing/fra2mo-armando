# 🤖 IK-BASED PICK & PLACE - GUIDA RAPIDA

Sistema pick&place semplificato con **cinematica inversa** per Armando.

## 📋 PREREQUISITI

### 1. Installa ikpy (libreria IK)
```bash
pip3 install ikpy
```

### 2. Rendi eseguibile lo script
```bash
cd ~/ros2_image_fold/fra2mo_armando
chmod +x scripts/ik_pick_place.py
```

### 3. Build del workspace
```bash
cd ~/ros2_image_fold/fra2mo_armando
colcon build
source install/setup.bash
```

---

## 🚀 UTILIZZO

### Metodo 1: Launch Completo (Gazebo + Task)
```bash
# Terminal 1: Avvia tutto
ros2 launch fra2mo_armando launch_world.launch.py
```

```bash
# Terminal 2: Lancia task (dopo che Gazebo è pronto)
ros2 launch fra2mo_armando ik_pick_place.launch.py
```

### Metodo 2: Solo Task (Gazebo già running)
```bash
# Se Gazebo è già attivo:
ros2 run fra2mo_armando ik_pick_place.py
```

---

## ⚙️ CONFIGURAZIONE

### Modifica Posizioni Pilastri

Apri `scripts/ik_pick_place.py` e modifica:

```python
# Line ~71-76
self.pillars = [
    {'x': 2.0, 'y': 1.5, 'name': 'pillar_1'},
    {'x': 2.0, 'y': -1.5, 'name': 'pillar_2'},
    {'x': -1.0, 'y': 2.0, 'name': 'pillar_3'},  # Aggiungi altri
]
```

### Modifica Altezze

```python
# Line ~63-66
self.ARM_BASE_HEIGHT = 0.24  # Altezza base braccio
self.PILLAR_HEIGHT = 0.60    # Altezza pilastro
self.OBJECT_HEIGHT = 0.05    # Altezza oggetto
self.BASKET_HEIGHT = 0.30    # Altezza basket
```

### Modifica Posizione Basket

```python
# Line ~79
self.basket_pos = {'x': -2.0, 'y': 0.0}
```

---

## 🔧 FUNZIONALITÀ

### ✅ Implementato:
- **Cinematica Inversa** con ikpy (manipolatore ridondante)
- **Sequenza Pick**:
  1. Calcolo posizione target oggetto (IK)
  2. Apertura gripper
  3. Movimento braccio verso oggetto
  4. Chiusura gripper
  5. Sollevamento
  6. Ritorno home
  
- **Sequenza Place**:
  1. Movimento sopra basket (IK)
  2. Rilascio oggetto
  3. Ritorno home

- **Task Iterativo**: Loop su tutti i pilastri configurati

### ⚠️ Da Implementare (TODO):
- **Navigazione autonoma** verso pilastri (Nav2)
- **Percezione LIDAR** per rilevare pilastri
- **Trasformazioni TF** precise map → base_link
- **Feedback visivo** (camera D435)

---

## 📊 OUTPUT ATTESO

```
=================================================
INIZIO TASK PICK & PLACE
=================================================

Posizione home iniziale...
Arm command: [0.0, 0.0, 0.0, 0.0, 0.03]

==================================================
Pilastro: pillar_1
Posizione: x=2.0, y=1.5
==================================================

=== PICK da pillar_1 ===
Target position (rel): [0.4  0.   0.41]
IK solution: [-0.123, 0.456, -0.789, 0.234]
Apertura gripper...
Movimento verso oggetto...
Chiusura gripper...
Sollevamento oggetto...
Ritorno a home position...

=== PLACE nel basket ===
Posizionamento sopra basket...
Rilascio oggetto...
Ritorno a home...

✓ pillar_1 completato!

[... ripete per altri pilastri ...]

==================================================
TASK COMPLETATO!
==================================================
```

---

## 🐛 TROUBLESHOOTING

### Errore "IK fallita!"
**Causa**: Posizione target fuori workspace braccio

**Soluzione**:
1. Verifica posizioni pilastri siano raggiungibili
2. Workspace braccio: ~0.2m-0.4m radius, altezza 0.3m-0.7m
3. Riduci distanza robot-pilastro

### Script non trovato
```bash
# Verifica installazione
ls install/fra2mo_armando/lib/fra2mo_armando/

# Rebuilda se manca
colcon build --packages-select fra2mo_armando
source install/setup.bash
```

### Movimenti non fluidi
**Causa**: Tempi di attesa fissi non sufficienti

**Soluzione**: Aumenta `time.sleep()` nelle sequenze (linee ~200-240)

---

## 🎯 PROSSIMI STEP SVILUPPO

### FASE 1: Navigazione (1-2 giorni)
- [ ] Integrazione Nav2 per waypoint navigation
- [ ] Topic `/navigate_to_pose` per pilastri
- [ ] Allineamento preciso con LIDAR

### FASE 2: Percezione (2-3 giorni)
- [ ] Clustering LIDAR per rilevare pilastri
- [ ] Stima posizione 3D automatica
- [ ] Camera D435 per verifica oggetti

### FASE 3: Sistema Completo (3-4 giorni)
- [ ] State machine robusta (SMACH)
- [ ] Error recovery
- [ ] Logging e diagnostica
- [ ] Test con setup variabili

---

## 📝 NOTE TECNICHE

### Catena Cinematica
Il braccio Armando ha:
- **4 DOF attivi**: j0, j1, j2, j3 (tutti revolute Z-axis)
- **6 link totali**: Include joint fissi intermedi
- **End-effector**: Centro gripper a ~0.05m da j3

### IK Solver
- Usa `ikpy.chain.Chain` con URDF-like definition
- Metodo: Jacobian-based numerical IK
- Convergenza: Tipicamente 10-50 iterazioni
- Tolleranza posizione: 1mm

### Trasformazioni (Semplificazione)
Attualmente il sistema assume:
- Robot allineato con pilastro (asse X verso pilastro)
- Coordinate trasformate manualmente
- **TODO**: Usare TF2 per trasformazioni automatiche

---

## 📞 SUPPORTO

Per problemi o miglioramenti, contatta il team di sviluppo.

**Versione**: 1.0 - IK-based simplified system  
**Data**: 2025-12-15  
**Autore**: Sistema AI + Giulio
