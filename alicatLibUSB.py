# -*- coding: utf-8 -*-
"""
AlicatController via USB/Série pour PCD Alicat
Protocole ASCII série (pyserial), sans dépendance à la lib alicat
Interface identique à alicatLib.py pour compatibilité avec AlicatGui

Modèle testé : PCD-1000PSIA-D-PCA41/5P
Réponse : b'A +01.104 +00.000 LCK\r'
           adresse  pression  setpoint  état

pip install pyserial
"""

import serial
import time
from typing import Optional, Dict, Any


class AlicatController:
    """
    Contrôleur Alicat PCD via port série USB (protocole ASCII).
    Interface identique à alicatLib.AlicatController (version EtherNet/IP).
    """

    def __init__(self, port: str = '/dev/ttyUSB0', address: str = 'A',
                 baudrate: int = 19200):
        """
        Args:
            port:     Port série, ex: '/dev/ttyUSB0' (Linux) ou 'COM3' (Windows)
            address:  Adresse de l'unité Alicat (défaut: 'A')
            baudrate: Vitesse série (défaut: 19200)
        """
        self.port = port
        self.address = address
        self.baudrate = baudrate
        self.connected = False
        self._serial: Optional[serial.Serial] = None

    # ------------------------------------------------------------------
    # Connexion / Déconnexion
    # ------------------------------------------------------------------

    def connect(self) -> bool:
        """Établit la connexion avec l'Alicat."""
        try:
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=8,
                parity='N',
                stopbits=1,
                timeout=0.5
            )
            # Vider les buffers résiduels
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()

            # Test de communication
            response = self._query(self.address)
            if response is None:
                raise RuntimeError("Pas de réponse de l'appareil")

            self.connected = True
            print(f"✓ Connexion établie avec Alicat PCD sur {self.port}")
            print(f"  Réponse initiale: {response}")
            return True

        except Exception as e:
            print(f"Erreur de connexion: {e}")
            if self._serial and self._serial.is_open:
                self._serial.close()
            return False

    def disconnect(self):
        """Ferme la connexion."""
        if self._serial and self._serial.is_open:
            self._serial.close()
        self.connected = False
        print("Connexion fermée")

    # ------------------------------------------------------------------
    # Communication série bas niveau
    # ------------------------------------------------------------------

    def _query(self, cmd: str) -> Optional[str]:
        """
        Envoie une commande et retourne la réponse.

        Args:
            cmd: Commande sans \\r (ex: 'A', 'AS5.0')
        Returns:
            Réponse décodée ou None
        """
        if self._serial is None or not self._serial.is_open:
            return None
        try:
            # Vider uniquement ce qui traîne AVANT d'écrire
            time.sleep(0.05)
            self._serial.reset_input_buffer()

            self._serial.write((cmd + '\r').encode('ascii'))

            # Lire caractère par caractère jusqu'au \r avec timeout global
            raw = b''
            deadline = time.time() + 1.0  # 1 seconde max
            while time.time() < deadline:
                if self._serial.in_waiting > 0:
                    c = self._serial.read(1)
                    raw += c
                    if c == b'\r':
                        break
                else:
                    time.sleep(0.01)

            if not raw:
                print("Pas de réponse (timeout)")
                return None

            return raw.decode('ascii', errors='ignore').strip()
        except Exception as e:
            print(f"Erreur communication: {e}")
            return None

    def _parse_response(self, response: str) -> Optional[Dict[str, Any]]:
        """
        Parse la réponse ASCII de l'Alicat PCD.

        Format: 'A +01.104 +00.000 LCK'
                 addr  pression setpoint état
        """
        if not response:
            return None
        try:
            parts = response.split()
            if len(parts) < 2:
                return None

            # La réponse peut commencer par l'adresse (lettre) ou directement par la pression
            # Format avec adresse : 'A +01.107 +05.000 LCK'
            # Format sans adresse : '+01.107 +05.000 LCK'
            try:
                float(parts[0])
                # parts[0] est un nombre -> pas d'adresse
                addr   = self.address
                pres   = float(parts[0])
                setp   = float(parts[1])
                state  = parts[2] if len(parts) > 2 else ''
            except ValueError:
                # parts[0] est une lettre -> adresse présente
                if len(parts) < 3:
                    return None
                addr   = parts[0]
                pres   = float(parts[1])
                setp   = float(parts[2])
                state  = parts[3] if len(parts) > 3 else ''

            return {
                'address':          addr,
                'pressure_primary': pres,
                'setpoint':         setp,
                'state':            state,
                # Champs complémentaires pour compatibilité alicatLib
                'pressure_secondary':  0.0,
                'pressure_barometric': 0.0,
                'temperature':         0.0,
                'flow_mass':           0.0,
                'flow_volumetric':     0.0,
                'totalizer_1':         0.0,
                'totalizer_2':         0.0,
                'humidity':            0.0,
                'valve_drive':         0.0,
                'alarm_state':         0,
                'device_status':       0,
                'gas_number':          0,
                'gas':                 'N/A',
                'control_point':       'pressure',
            }
        except (ValueError, IndexError) as e:
            print(f"Erreur parsing '{response}': {e}")
            return None

    # ------------------------------------------------------------------
    # API publique (compatible alicatLib)
    # ------------------------------------------------------------------

    def get_readings(self) -> Optional[Dict[str, Any]]:
        """Lit toutes les données de l'Alicat."""
        if not self.connected:
            return None
        response = self._query(self.address)
        return self._parse_response(response)

    def get_pressure(self) -> float:
        """Retourne la pression primaire en Bar."""
        try:
            readings = self.get_readings()
            return readings['pressure_primary'] if readings else 0.0
        except Exception:
            return 0.0

    def set_setpoint(self, setpoint: float) -> bool:
        """
        Définit le setpoint de pression.

        Args:
            setpoint: Valeur de consigne en Bar
        Returns:
            True si succès
        """
        if not self.connected:
            return False
        try:
            cmd = f"{self.address}S{setpoint:.3f}"
            response = self._query(cmd)
            if response is None:
                return False
            parsed = self._parse_response(response)
            if parsed and abs(parsed['setpoint'] - setpoint) < 0.01:
                return True
            # L'appareil a quand même répondu, on considère OK
            return parsed is not None
        except Exception as e:
            print(f"Erreur setpoint: {e}")
            return False

    def get_device_info(self) -> Optional[Dict[str, Any]]:
        """
        Retourne les informations du dispositif.
        Format compatible avec alicatLib.get_device_info().
        """
        if not self.connected:
            return None
        return {
            'vendor_id':    1174,
            'device_type':  0,
            'product_code': 0,
            'status':       0,
            'serial_number': self.port,
            'product_name': f"Alicat PCD @ {self.port}",
        }

    def display_readings(self, readings: Dict[str, Any]):
        """Affiche les lectures de manière formatée."""
        if not readings:
            print("Aucune lecture disponible")
            return
        print("\n=== LECTURES ALICAT PCD (USB) ===")
        print(f"Pressure:  {readings['pressure_primary']:.3f} Bar")
        print(f"Setpoint:  {readings['setpoint']:.3f} Bar")
        print(f"State:     {readings.get('state', 'N/A')}")

    def _query_register(self, reg: int) -> str | None:
        """
        Lit un registre Alicat via $$r.
        Retourne la valeur brute (champ [3] de la réponse), ou None.
        """
        response = self._query(f"{self.address}$$r{reg}")
        if not response:
            return None
        try:
            return response.split()[3]
        except IndexError:
            print(f"[_query_register] Parsing echoue: '{response}'")
            return None

    def get_pid(self) -> dict | None:
        """
        Lit les paramètres PID depuis l'Alicat PCD.

        Returns:
            dict avec clés : loop_type ('PD/PDF' ou 'PD2I'), P, D, I
            ou None en cas d'erreur.
        """
        if not self.connected:
            return None

        # --- Loop type (registre 85) ---
        raw_loop = self._query_register(85)
        if raw_loop is None:
            print("[get_pid] Echec lecture loop_type (reg 85)")
            return None
        try:
            loopnum = int(raw_loop)
            loop_type = ['PD/PDF', 'PD/PDF', 'PD2I'][loopnum]   # 0->PD/PDF, 1->PD/PDF, 2->PD2I
        except (ValueError, IndexError):
            loop_type = 'PD/PDF'

        # --- P (21), D (22), I (23) ---
        pid = {'loop_type': loop_type}
        for key, reg in [('P', 21), ('D', 22), ('I', 23)]:
            raw = self._query_register(reg)
            if raw is None:
                print(f"[get_pid] Echec lecture registre {reg} ({key})")
                return None
            pid[key] = str(raw)   # retourné comme str, cohérent avec la lib officielle
            time.sleep(0.05)

        return pid


    def set_pid(self, P=None, I=None, D=None, loop_type=None) -> bool:
        """
        Ecrit les paramètres PID dans l'Alicat PCD.

        Args:
            P         : Gain proportionnel (int ou float)
            I         : Gain intégral — seulement utilisé en mode PD2I
            D         : Gain dérivé
            loop_type : 'PD/PDF'  ou  'PD2I'
        Returns:
            True si toutes les écritures ont réussi.
        """
        if not self.connected:
            return False

        # --- Loop type ---
        if loop_type is not None:
            options = ['PD/PDF', 'PD2I']
            if loop_type not in options:
                print(f"[set_pid] loop_type invalide: {loop_type}. Options: {options}")
                return False
            loop_num = options.index(loop_type) + 1   # PD/PDF->1, PD2I->2
            resp = self._query(f"{self.address}$$w85={loop_num}")
            if not resp:
                print("[set_pid] Echec écriture loop_type (reg 85)")
                return False
            time.sleep(0.05)

        # --- P (21) ---
        if P is not None:
            resp = self._query(f"{self.address}$$w21={int(P)}")
            if not resp:
                print("[set_pid] Echec écriture P (reg 21)")
                return False
            time.sleep(0.05)

        # --- D (22) ---  attention: D=22, I=23 dans l'ordre Alicat
        if D is not None:
            resp = self._query(f"{self.address}$$w22={int(D)}")
            if not resp:
                print("[set_pid] Echec écriture D (reg 22)")
                return False
            time.sleep(0.05)

        # --- I (23) ---
        if I is not None:
            resp = self._query(f"{self.address}$$w23={int(I)}")
            if not resp:
                print("[set_pid] Echec écriture I (reg 23)")
                return False
            time.sleep(0.05)

        return True

# ------------------------------------------------------------------
# Test standalone
# ------------------------------------------------------------------

if __name__ == '__main__':
    alicat = AlicatController('/dev/ttyUSB0')

    if alicat.connect():
        readings = alicat.get_readings()
        if readings:
            alicat.display_readings(readings)

        print("\nTest setpoint à 2.0 Bar...")
        alicat.set_setpoint(2.0)
        time.sleep(0.5)
        readings = alicat.get_readings()
        if readings:
            alicat.display_readings(readings)

        alicat.disconnect()