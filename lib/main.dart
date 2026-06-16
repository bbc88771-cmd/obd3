// OBD Scanner — единый файл (BLE + Wi-Fi).
// Всё приложение в одном файле, чтобы исключить путаницу между файлами.
import 'dart:async';
import 'dart:convert';
import 'dart:io';
import 'dart:math';
import 'package:flutter/material.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';
import 'package:permission_handler/permission_handler.dart';



// ===== из obd_transport.dart =====

/// Состояние соединения. Используется конечным автоматом в Riverpod.
enum LinkState { disconnected, scanning, connecting, initializing, connected, error }

/// Единый контракт для всех физических каналов связи с ELM327.
///
/// Благодаря этой абстракции доменный слой (ObdService) НЕ знает,
/// через что он говорит с адаптером — BLE, Bluetooth Classic или Wi-Fi.
/// Чтобы добавить новый транспорт — достаточно реализовать этот интерфейс.
abstract class ObdTransport {
  /// Установить физическое соединение (скан + подключение).
  Future<void> connect();

  /// Разорвать соединение и освободить ресурсы.
  Future<void> disconnect();

  /// Отправить ASCII-команду адаптеру.
  /// Завершающий символ \r (0x0D) добавляет сама реализация.
  Future<void> write(String command);

  /// Сырой поток байтов ОТ адаптера. Приходит кусками (chunks),
  /// сборкой целого ответа занимается слой выше (ObdConnection).
  Stream<List<int>> get incoming;

  bool get isConnected;

  /// Поток изменений состояния линка (для авто-reconnect и UI).
  Stream<LinkState> get linkState;
}


// ===== из obd_parsers.dart =====

/// Чистые функции разбора HEX-ответов ЭБУ. Без состояния — легко тестировать.
///
/// Общая логика OBD-II:
///   запрос  = <Mode><PID>           напр. "010C"  (Mode 01, PID 0C)
///   ответ   = <Mode+0x40><PID><данные A B C ...>
///             напр. "410C1AF8" → Mode 41 (=01+0x40), PID 0C, A=1A, B=F8
class ObdParsers {
  /// Маркеры мусорных/служебных ответов адаптера.
  static bool _isGarbage(String r) {
    return r.contains("NODATA") ||
        r.contains("ERROR") ||
        r.contains("SEARCHING") ||
        r.contains("UNABLE") ||
        r.contains("STOPPED") ||
        r.contains("BUFFERFULL") ||
        r.contains("CANERROR") ||
        r.isEmpty;
  }

  /// Достаёт байты данных (A, B, ...) из ответа, проверяя эхо Mode+PID.
  /// Возвращает null, если ответ битый или не тот PID.
  static List<int>? extractData(String resp, String mode, String pid) {
    if (_isGarbage(resp)) return null;

    // режем строку на пары символов = байты
    final bytes = <int>[];
    for (int i = 0; i + 2 <= resp.length; i += 2) {
      final b = int.tryParse(resp.substring(i, i + 2), radix: 16);
      if (b == null) return null; // мусорный символ
      bytes.add(b);
    }

    final modeByte = int.parse(mode, radix: 16) + 0x40; // 01 -> 41
    final pidByte = int.parse(pid, radix: 16);

    // ищем начало валидного фрейма (может быть мусор в начале строки)
    for (int i = 0; i + 1 < bytes.length; i++) {
      if (bytes[i] == modeByte && bytes[i + 1] == pidByte) {
        return bytes.sublist(i + 2); // только полезная нагрузка
      }
    }
    return null;
  }

  /// RPM (PID 0C): (A*256 + B) / 4.
  /// Два байта, разрешение ¼ об/мин → диапазон 0..16383.75.
  /// Пример: A=0x1A(26), B=0xF8(248) → (26*256+248)/4 = 6904/4 = 1726 RPM.
  static int? rpm(String resp) {
    final d = extractData(resp, "01", "0C");
    if (d == null || d.length < 2) return null;
    final v = ((d[0] * 256) + d[1]) ~/ 4;
    return (v >= 0 && v <= 16383) ? v : null; // sanity-check
  }

  /// Скорость (PID 0D): A км/ч напрямую. Один байт 0..255.
  static int? speed(String resp) {
    final d = extractData(resp, "01", "0D");
    if (d == null || d.isEmpty) return null;
    final v = d[0];
    return (v <= 255) ? v : null;
  }

  /// Температура ОЖ (PID 05): A - 40 (°C).
  /// Смещение −40 позволяет кодировать минус без знакового байта
  /// (байт 0x00 = −40 °C, 0xFF = 215 °C).
  static int? coolant(String resp) {
    final d = extractData(resp, "01", "05");
    if (d == null || d.isEmpty) return null;
    return d[0] - 40;
  }

  /// Нагрузка двигателя (PID 04): A * 100 / 255 (%).
  static double? engineLoad(String resp) {
    final d = extractData(resp, "01", "04");
    if (d == null || d.isEmpty) return null;
    return d[0] * 100 / 255;
  }

  /// Положение дросселя (PID 11): A * 100 / 255 (%).
  static double? throttle(String resp) {
    final d = extractData(resp, "01", "11");
    if (d == null || d.isEmpty) return null;
    return d[0] * 100 / 255;
  }

  /// Напряжение по команде ELM327 "ATRV" (не OBD-PID, особый ответ "12.3V").
  static double? voltage(String resp) {
    final clean = resp.replaceAll("V", "");
    return double.tryParse(clean);
  }

  // ─────────────── РАСШИРЕННЫЕ PID ───────────────

  /// Температура впускного воздуха (PID 0F): A - 40 (°C). Та же логика смещения.
  static int? intakeTemp(String resp) {
    final d = extractData(resp, "01", "0F");
    if (d == null || d.isEmpty) return null;
    return d[0] - 40;
  }

  /// Уровень топлива в баке (PID 2F): A * 100 / 255 (%).
  static double? fuelLevel(String resp) {
    final d = extractData(resp, "01", "2F");
    if (d == null || d.isEmpty) return null;
    return d[0] * 100 / 255;
  }

  /// Расход воздуха MAF (PID 10): (A*256 + B) / 100 (грамм/сек).
  /// Два байта, разрешение 0.01 г/с. Нужен для расчёта мгновенного расхода топлива.
  static double? maf(String resp) {
    final d = extractData(resp, "01", "10");
    if (d == null || d.length < 2) return null;
    return ((d[0] * 256) + d[1]) / 100;
  }

  /// Давление во впускном коллекторе MAP (PID 0B): A кПа напрямую (абсолютное).
  /// Для турбо: наддув = MAP - атмосферное(~101 кПа).
  static int? mapPressure(String resp) {
    final d = extractData(resp, "01", "0B");
    if (d == null || d.isEmpty) return null;
    return d[0];
  }

  /// Давление топлива (PID 0A): A * 3 (кПа). Множитель 3 = разрешение датчика.
  static int? fuelPressure(String resp) {
    final d = extractData(resp, "01", "0A");
    if (d == null || d.isEmpty) return null;
    return d[0] * 3;
  }

  /// Угол опережения зажигания (PID 0E): A/2 - 64 (°).
  /// Смещение −64 позволяет кодировать отрицательные углы (запаздывание).
  static double? timingAdvance(String resp) {
    final d = extractData(resp, "01", "0E");
    if (d == null || d.isEmpty) return null;
    return d[0] / 2 - 64;
  }

  /// Температура окружающего воздуха (PID 46): A - 40 (°C).
  static int? ambientTemp(String resp) {
    final d = extractData(resp, "01", "46");
    if (d == null || d.isEmpty) return null;
    return d[0] - 40;
  }

  /// Обороты по двум байтам — общий помощник, если нужен другой PID с той же формулой.
  /// Лямбда O2 (PID 24, широкополосный): ((A*256+B)/65536)*2 — коэффициент эквивалентности.
  static double? lambda(String resp) {
    final d = extractData(resp, "01", "24");
    if (d == null || d.length < 2) return null;
    return ((d[0] * 256) + d[1]) / 65536 * 2;
  }

  /// Пробег с момента сброса ошибок (PID 31): A*256 + B (км).
  static int? distanceSinceClear(String resp) {
    final d = extractData(resp, "01", "31");
    if (d == null || d.length < 2) return null;
    return (d[0] * 256) + d[1];
  }

  /// Расчётный мгновенный расход топлива (л/ч) из MAF.
  /// Формула для бензина (AFR≈14.7, плотность≈0.745 кг/л):
  ///   л/ч = (MAF[г/с] * 3600) / (14.7 * 745)
  static double? fuelRateFromMaf(double mafGramsPerSec) {
    return (mafGramsPerSec * 3600) / (14.7 * 745);
  }

  /// Чтение DTC (Mode 03). Каждый код = 2 байта.
  ///   старшие 2 бита 1-го байта → буква системы (P/C/B/U)
  ///   биты 5-4 → первая цифра, биты 3-0 → вторая цифра
  ///   2-й байт → последние две шестнадцатеричные цифры
  /// Пример: байты 01 33 → P0133.
  static List<String> dtc(String resp) {
    if (_isGarbage(resp) || !resp.startsWith("43")) return [];
    final hex = resp.substring(2); // отрезаем эхо режима "43"
    final codes = <String>[];
    const letters = ['P', 'C', 'B', 'U'];

    for (int i = 0; i + 4 <= hex.length; i += 4) {
      final chunk = hex.substring(i, i + 4);
      if (chunk == "0000") continue; // пустой слот

      final first = int.parse(chunk.substring(0, 2), radix: 16);
      final letter = letters[(first & 0xC0) >> 6];   // биты 7-6
      final d1 = (first & 0x30) >> 4;                // биты 5-4
      final d2 = first & 0x0F;                       // биты 3-0
      final rest = chunk.substring(2);               // байт B как есть

      codes.add("$letter$d1${d2.toRadixString(16)}$rest".toUpperCase());
    }
    return codes;
  }
}

/// Каталог PID для удобного опроса.
class Pid {
  final String label;
  final String cmd;     // что слать, напр. "010C"
  final String unit;
  const Pid(this.label, this.cmd, this.unit);

  static const rpm = Pid("Обороты", "010C", "об/мин");
  static const speed = Pid("Скорость", "010D", "км/ч");
  static const coolant = Pid("Темп. ОЖ", "0105", "°C");
  static const load = Pid("Нагрузка", "0104", "%");
  static const throttle = Pid("Дроссель", "0111", "%");

  // расширенные
  static const intakeTemp = Pid("Темп. впуска", "010F", "°C");
  static const fuelLevel = Pid("Топливо", "012F", "%");
  static const maf = Pid("MAF", "0110", "г/с");
  static const map = Pid("Наддув (MAP)", "010B", "кПа");
  static const fuelPressure = Pid("Давл. топлива", "010A", "кПа");
  static const timing = Pid("Опереж. зажиг.", "010E", "°");
  static const ambient = Pid("Темп. за бортом", "0146", "°C");
  static const lambda = Pid("Лямбда", "0124", "λ");
  static const distance = Pid("Пробег с DTC", "0131", "км");
}


// ===== из obd_connection.dart =====

/// Превращает «рваный» поток байтов от адаптера в цельные текстовые ответы.
///
/// ELM327 завершает КАЖДЫЙ ответ символом-приглашением '>' (0x3E).
/// Байты могут приходить кусками ("41 0C", потом "1A F8\r>"),
/// поэтому копим буфер до появления '>'.
class ObdConnection {
  final ObdTransport transport;
  final StringBuffer _buffer = StringBuffer();
  Completer<String>? _pending;

  late final StreamSubscription _sub;

  ObdConnection(this.transport) {
    _sub = transport.incoming.listen(_onBytes);
  }

  void _onBytes(List<int> chunk) {
    _buffer.write(String.fromCharCodes(chunk));
    final current = _buffer.toString();

    if (current.contains('>')) {
      _buffer.clear();
      // нормализация: убираем приглашение, переводы строк и пробелы.
      // (ATS0 обычно уже убрал пробелы, но подстраховываемся)
      final clean = current
          .replaceAll('>', '')
          .replaceAll('\r', '')
          .replaceAll('\n', '')
          .replaceAll(' ', '')
          .toUpperCase()
          .trim();

      final p = _pending;
      _pending = null;
      if (p != null && !p.isCompleted) p.complete(clean);
    }
  }

  /// Отправляет одну команду и ждёт цельный ответ или таймаут.
  Future<String> send(
    String cmd, {
    Duration timeout = const Duration(milliseconds: 1500),
  }) {
    _buffer.clear(); // чистим хвосты предыдущего обмена
    final completer = Completer<String>();
    _pending = completer;

    transport.write(cmd);

    return completer.future.timeout(timeout, onTimeout: () {
      _pending = null;
      throw TimeoutException("Таймаут ответа на '$cmd'");
    });
  }

  Future<void> dispose() async => _sub.cancel();
}


// ===== из request_queue.dart =====

/// Последовательная очередь команд.
///
/// ELM327 полудуплексный и обрабатывает РОВНО одну команду за раз.
/// Если послать вторую до прихода '>', ответы слипнутся в кашу.
/// Очередь гарантирует строгий порядок «запрос → ответ → следующий».
class RequestQueue {
  final ObdConnection conn;
  final List<_Item> _items = [];
  bool _pumping = false;

  /// Микропауза между командами — дешёвым клонам нужно «продохнуть».
  final Duration gap;

  RequestQueue(this.conn, {this.gap = const Duration(milliseconds: 25)});

  /// Поставить команду в очередь. Возвращает future с её ответом.
  Future<String> enqueue(String cmd, {Duration? timeout}) {
    final item = _Item(cmd, Completer<String>(), timeout);
    _items.add(item);
    _pump(); // запускаем «насос», если стоял
    return item.completer.future;
  }

  Future<void> _pump() async {
    if (_pumping) return;
    _pumping = true;

    while (_items.isNotEmpty) {
      final item = _items.removeAt(0);
      try {
        final resp = item.timeout != null
            ? await conn.send(item.cmd, timeout: item.timeout!)
            : await conn.send(item.cmd);
        if (!item.completer.isCompleted) item.completer.complete(resp);
      } catch (e) {
        // один таймаут НЕ должен ронять всю очередь
        if (!item.completer.isCompleted) item.completer.completeError(e);
      }
      await Future.delayed(gap);
    }

    _pumping = false;
  }

  void clear() {
    for (final i in _items) {
      if (!i.completer.isCompleted) {
        i.completer.completeError(StateError("Очередь очищена"));
      }
    }
    _items.clear();
  }
}

class _Item {
  final String cmd;
  final Completer<String> completer;
  final Duration? timeout;
  _Item(this.cmd, this.completer, this.timeout);
}


// ===== из obd_service.dart =====

/// Снимок телеметрии для UI.
class Telemetry {
  final int? rpm;
  final int? speed;
  final int? coolant;
  final double? load;
  final double? voltage;
  // расширенные
  final int? intakeTemp;
  final double? fuelLevel;
  final double? maf;
  final int? map;
  final double? timing;
  final int? ambient;
  final double? fuelRate; // л/ч, расчётный из MAF

  const Telemetry({
    this.rpm,
    this.speed,
    this.coolant,
    this.load,
    this.voltage,
    this.intakeTemp,
    this.fuelLevel,
    this.maf,
    this.map,
    this.timing,
    this.ambient,
    this.fuelRate,
  });

  Telemetry copyWith({
    int? rpm,
    int? speed,
    int? coolant,
    double? load,
    double? voltage,
    int? intakeTemp,
    double? fuelLevel,
    double? maf,
    int? map,
    double? timing,
    int? ambient,
    double? fuelRate,
  }) =>
      Telemetry(
        rpm: rpm ?? this.rpm,
        speed: speed ?? this.speed,
        coolant: coolant ?? this.coolant,
        load: load ?? this.load,
        voltage: voltage ?? this.voltage,
        intakeTemp: intakeTemp ?? this.intakeTemp,
        fuelLevel: fuelLevel ?? this.fuelLevel,
        maf: maf ?? this.maf,
        map: map ?? this.map,
        timing: timing ?? this.timing,
        ambient: ambient ?? this.ambient,
        fuelRate: fuelRate ?? this.fuelRate,
      );
}

/// Главный доменный слой: инициализация адаптера, цикл опроса,
/// чтение/сброс ошибок и контроль безопасности.
class ObdService {
  final ObdTransport transport;
  late final ObdConnection _conn;
  late final RequestQueue _queue;

  final _telemetry = StreamController<Telemetry>.broadcast();
  Stream<Telemetry> get telemetry => _telemetry.stream;
  Telemetry _last = const Telemetry();

  Timer? _pollTimer;
  int _consecutiveTimeouts = 0;

  ObdService(this.transport) {
    _conn = ObdConnection(transport);
    _queue = RequestQueue(_conn);
  }

  /// Полная инициализация ELM327. Порядок команд важен.
  Future<void> initialize() async {
    // ATZ — полный сброс, адаптер «думает» ~1 сек, даём больше времени.
    await _queue.enqueue("ATZ", timeout: const Duration(seconds: 3));
    await _queue.enqueue("ATE0"); // Echo Off — ответ без копии нашей команды
    await _queue.enqueue("ATL0"); // Linefeed Off — без лишних \n
    await _queue.enqueue("ATS0"); // Spaces Off — компактный HEX "410C1AF8"
    await _queue.enqueue("ATH0"); // Headers Off — на этапе значений не нужны
    await _queue.enqueue("ATSP0"); // авто-определение протокола авто
    // «пробный» OBD-запрос — заставляет адаптер реально выбрать протокол
    await _queue.enqueue("0100", timeout: const Duration(seconds: 5));
    // выясняем, какие PID реально поддерживает ЭБУ этого авто
    await detectSupportedPids();
  }

  /// Запустить периодический опрос (poll loop) с заданной частотой.
  void startPolling({Duration interval = const Duration(milliseconds: 200)}) {
    _pollTimer?.cancel();
    _pollTimer = Timer.periodic(interval, (_) => _pollOnce());
  }

  void stopPolling() => _pollTimer?.cancel();

  Future<void> _pollOnce() async {
    // Все команды идут через очередь, поэтому не накладываются.
    await _read(Pid.rpm.cmd, (r) {
      final v = ObdParsers.rpm(r);
      if (v != null) _emit(_last.copyWith(rpm: v));
    });
    await _read(Pid.speed.cmd, (r) {
      final v = ObdParsers.speed(r);
      if (v != null) _emit(_last.copyWith(speed: v));
    });
    await _read(Pid.coolant.cmd, (r) {
      final v = ObdParsers.coolant(r);
      if (v != null) _emit(_last.copyWith(coolant: v));
    });
    await _read(Pid.load.cmd, (r) {
      final v = ObdParsers.engineLoad(r);
      if (v != null) _emit(_last.copyWith(load: v));
    });
    await _read("ATRV", (r) {
      final v = ObdParsers.voltage(r);
      if (v != null) _emit(_last.copyWith(voltage: v));
    });

    // расширенные PID опрашиваем только если ЭБУ их поддерживает
    // (проверка через 0100/0120 — список поддержки, см. _detectSupported)
    if (_supports("0F")) {
      await _read(Pid.intakeTemp.cmd, (r) {
        final v = ObdParsers.intakeTemp(r);
        if (v != null) _emit(_last.copyWith(intakeTemp: v));
      });
    }
    if (_supports("2F")) {
      await _read(Pid.fuelLevel.cmd, (r) {
        final v = ObdParsers.fuelLevel(r);
        if (v != null) _emit(_last.copyWith(fuelLevel: v));
      });
    }
    if (_supports("10")) {
      await _read(Pid.maf.cmd, (r) {
        final v = ObdParsers.maf(r);
        if (v != null) {
          // попутно считаем мгновенный расход топлива
          final rate = ObdParsers.fuelRateFromMaf(v);
          _emit(_last.copyWith(maf: v, fuelRate: rate));
        }
      });
    }
    if (_supports("0B")) {
      await _read(Pid.map.cmd, (r) {
        final v = ObdParsers.mapPressure(r);
        if (v != null) _emit(_last.copyWith(map: v));
      });
    }
    if (_supports("0E")) {
      await _read(Pid.timing.cmd, (r) {
        final v = ObdParsers.timingAdvance(r);
        if (v != null) _emit(_last.copyWith(timing: v));
      });
    }
    if (_supports("46")) {
      await _read(Pid.ambient.cmd, (r) {
        final v = ObdParsers.ambientTemp(r);
        if (v != null) _emit(_last.copyWith(ambient: v));
      });
    }
  }

  // ─────────── автоопределение поддерживаемых PID ───────────
  // ЭБУ на запрос 0100 возвращает битовую маску: какие PID 01-20 поддержаны.
  final Set<String> _supported = {};

  bool _supports(String pid) => _supported.contains(pid.toUpperCase());

  /// Запрашивает маски поддержки (0100, 0120, 0140) и заполняет _supported.
  /// Вызывается один раз после initialize().
  Future<void> detectSupportedPids() async {
    for (final base in ["0100", "0120", "0140"]) {
      try {
        final resp = await _queue.enqueue(base);
        _parseSupportMask(resp, base);
      } catch (_) {/* блок не поддержан — ок */}
    }
  }

  void _parseSupportMask(String resp, String base) {
    // ответ вида 41 00 BE 1F A8 13 → 4 байта маски = 32 бита = PID 01..20
    final pidByte = base.substring(2); // "00","20","40"
    final d = ObdParsers.extractData(resp, "01", pidByte);
    if (d == null || d.length < 4) return;

    final offset = int.parse(pidByte, radix: 16); // 0x00, 0x20, 0x40
    int bit = 0;
    for (final byte in d.take(4)) {
      for (int i = 7; i >= 0; i--) {
        bit++;
        if ((byte & (1 << i)) != 0) {
          final pidNum = offset + bit;
          _supported.add(pidNum.toRadixString(16).padLeft(2, '0').toUpperCase());
        }
      }
    }
  }

  Future<void> _read(String cmd, void Function(String) onOk) async {
    try {
      final resp = await _queue.enqueue(cmd);
      _consecutiveTimeouts = 0; // успех — сбрасываем счётчик
      onOk(resp);
    } on TimeoutException {
      _consecutiveTimeouts++;
      // 3 таймаута подряд → линк, скорее всего, умер
      if (_consecutiveTimeouts >= 3) {
        stopPolling();
        _telemetry.addError(StateError("Соединение потеряно"));
      }
    } catch (_) {/* битый ответ — просто пропускаем кадр */}
  }

  void _emit(Telemetry t) {
    _last = t;
    _telemetry.add(t);
  }

  /// Прочитать сохранённые коды ошибок.
  Future<List<String>> readDtc() async {
    final resp = await _queue.enqueue("03", timeout: const Duration(seconds: 3));
    return ObdParsers.dtc(resp);
  }

  /// БЕЗОПАСНОСТЬ: проверка, что машина неподвижна перед записью.
  Future<bool> _isSafeToWrite() async {
    try {
      final resp = await _queue.enqueue(Pid.speed.cmd);
      final speed = ObdParsers.speed(resp);
      // нет данных → считаем НЕбезопасным (fail-safe)
      return speed == 0;
    } catch (_) {
      return false;
    }
  }

  /// Сброс DTC (Mode 04). Только на стоящей машине + подтверждение в UI.
  /// Гасит Check Engine и стирает freeze frame — необратимо.
  Future<void> clearDtc({required bool userConfirmed}) async {
    if (!userConfirmed) {
      throw Exception("Требуется подтверждение пользователя");
    }
    if (!await _isSafeToWrite()) {
      throw Exception("Сброс ошибок запрещён: автомобиль должен стоять (V=0)");
    }
    await _queue.enqueue("04", timeout: const Duration(seconds: 3));
  }

  Future<void> dispose() async {
    stopPolling();
    _queue.clear();
    await _conn.dispose();
    await _telemetry.close();
  }
}


// ===== из ble_transport.dart =====

/// Транспорт для ELM327-адаптеров с Bluetooth Low Energy.
///
/// Большинство дешёвых BLE-клонов (Vgate iCar, Veepeak, vLinker) используют
/// сервис FFE0 с одной характеристикой FFE1, которая одновременно
/// принимает запись и шлёт notify. Иногда встречается раздельная пара
/// (write FFE1 + notify FFE2) или сервис 18F0/FFF0 — поэтому ниже идёт
/// перебор кандидатов, а не жёсткая привязка.
class BleTransport implements ObdTransport {
  BluetoothDevice? _device;
  BluetoothCharacteristic? _writeChar;
  BluetoothCharacteristic? _notifyChar;

  StreamSubscription? _notifySub;
  StreamSubscription? _connSub;

  final _incoming = StreamController<List<int>>.broadcast();
  final _state = StreamController<LinkState>.broadcast();

  @override
  Stream<List<int>> get incoming => _incoming.stream;
  @override
  Stream<LinkState> get linkState => _state.stream;
  @override
  bool get isConnected => _device?.isConnected ?? false;

  // Известные UUID сервисов ELM327-BLE клонов (16-битные, развёрнутые в 128).
  static final _candidateServices = <Guid>[
    Guid("0000ffe0-0000-1000-8000-00805f9b34fb"),
    Guid("0000fff0-0000-1000-8000-00805f9b34fb"),
    Guid("000018f0-0000-1000-8000-00805f9b34fb"),
  ];

  /// Сканируем эфир, ищем устройство с «обдшным» именем.
  Future<BluetoothDevice?> _scan() async {
    _state.add(LinkState.scanning);
    BluetoothDevice? found;

    final sub = FlutterBluePlus.scanResults.listen((results) {
      for (final r in results) {
        final name = r.device.platformName.toUpperCase();
        if (name.contains("OBD") ||
            name.contains("VLINK") ||
            name.contains("VEEPEAK") ||
            name.contains("ELM")) {
          found = r.device;
        }
      }
    });

    await FlutterBluePlus.startScan(timeout: const Duration(seconds: 8));
    await FlutterBluePlus.isScanning.where((s) => s == false).first;
    await sub.cancel();
    return found;
  }

  @override
  Future<void> connect() async {
    _device = await _scan();
    if (_device == null) {
      _state.add(LinkState.error);
      throw Exception("BLE OBD-адаптер не найден. Включи зажигание и Bluetooth.");
    }

    _state.add(LinkState.connecting);

    // следим за разрывом для авто-reconnect / UI
    _connSub = _device!.connectionState.listen((s) {
      if (s == BluetoothConnectionState.disconnected) {
        _state.add(LinkState.disconnected);
      }
    });

    await _device!.connect(timeout: const Duration(seconds: 12));

    // некоторым клонам нужен запрос увеличенного MTU,
    // иначе длинные ответы (VIN, multiline DTC) режутся
    try {
      await _device!.requestMtu(180);
    } catch (_) {/* не критично */}

    await _discoverCharacteristics();
    _state.add(LinkState.connected);
  }

  Future<void> _discoverCharacteristics() async {
    final services = await _device!.discoverServices();

    for (final svc in services) {
      if (!_candidateServices.contains(svc.uuid)) continue;

      for (final c in svc.characteristics) {
        final props = c.properties;
        if (props.write || props.writeWithoutResponse) _writeChar = c;
        if (props.notify || props.indicate) _notifyChar = c;
      }
    }

    // фолбэк: если по белому списку не нашли — берём любую подходящую пару
    if (_writeChar == null || _notifyChar == null) {
      for (final svc in services) {
        for (final c in svc.characteristics) {
          if ((c.properties.write || c.properties.writeWithoutResponse) &&
              _writeChar == null) {
            _writeChar = c;
          }
          if ((c.properties.notify || c.properties.indicate) &&
              _notifyChar == null) {
            _notifyChar = c;
          }
        }
      }
    }

    if (_writeChar == null || _notifyChar == null) {
      throw Exception("Не найдены характеристики для обмена данными");
    }

    await _notifyChar!.setNotifyValue(true);
    _notifySub = _notifyChar!.onValueReceived.listen((bytes) {
      _incoming.add(bytes); // отдаём сырые байты наверх
    });
  }

  @override
  Future<void> write(String command) async {
    if (_writeChar == null) throw Exception("Нет канала записи");
    // ВАЖНО: каждая команда ОБЯЗАНА заканчиваться \r, иначе ELM327 её игнорит.
    final data = utf8.encode("$command\r");
    // writeWithoutResponse быстрее; OBD-команды короткие и влезают в MTU.
    await _writeChar!.write(
      data,
      withoutResponse: _writeChar!.properties.writeWithoutResponse,
    );
  }

  @override
  Future<void> disconnect() async {
    await _notifySub?.cancel();
    await _connSub?.cancel();
    await _device?.disconnect();
    _state.add(LinkState.disconnected);
  }
}


// ===== из wifi_transport.dart =====

/// Транспорт для Wi-Fi ELM327-адаптеров.
///
/// Такие адаптеры поднимают собственную точку доступа и работают как
/// TCP-сервер. Классический дефолт у клонов: 192.168.0.10 порт 35000.
/// Телефон должен быть подключён к Wi-Fi-сети самого адаптера.
class WifiTransport implements ObdTransport {
  final String host;
  final int port;

  Socket? _socket;
  StreamSubscription? _sub;

  final _incoming = StreamController<List<int>>.broadcast();
  final _state = StreamController<LinkState>.broadcast();

  WifiTransport({this.host = "192.168.0.10", this.port = 35000});

  @override
  Stream<List<int>> get incoming => _incoming.stream;
  @override
  Stream<LinkState> get linkState => _state.stream;
  @override
  bool get isConnected => _socket != null;

  @override
  Future<void> connect() async {
    _state.add(LinkState.connecting);
    try {
      _socket = await Socket.connect(
        host,
        port,
        timeout: const Duration(seconds: 8),
      );

      // поток байтов из сокета — тот же контракт, что и у BLE
      _sub = _socket!.listen(
        (bytes) => _incoming.add(bytes),
        onError: (_) => _state.add(LinkState.error),
        onDone: () {
          _socket = null;
          _state.add(LinkState.disconnected);
        },
      );

      _state.add(LinkState.connected);
    } catch (e) {
      _state.add(LinkState.error);
      throw Exception("Не удалось подключиться к $host:$port. "
          "Проверь, что телефон в Wi-Fi-сети адаптера.");
    }
  }

  @override
  Future<void> write(String command) async {
    if (_socket == null) throw Exception("Сокет закрыт");
    _socket!.add(utf8.encode("$command\r")); // \r обязателен
    await _socket!.flush();
  }

  @override
  Future<void> disconnect() async {
    await _sub?.cancel();
    await _socket?.close();
    _socket = null;
    _state.add(LinkState.disconnected);
  }
}


// ===== из connection_provider.dart =====

/// Тип транспорта, выбранный пользователем.
enum TransportKind { ble, btClassic, wifi }

final transportKindProvider = StateProvider<TransportKind>((ref) {
  // на iOS BT Classic недоступен — по умолчанию BLE
  return TransportKind.ble;
});

/// Фабрика транспорта по выбранному типу.
ObdTransport _buildTransport(TransportKind kind) {
  switch (kind) {
    case TransportKind.ble:
      return BleTransport();
    case TransportKind.wifi:
      return WifiTransport(); // 192.168.0.10:35000 по умолчанию
    case TransportKind.btClassic:
      return BleTransport(); // BT Classic убран
  }
}

/// Конечный автомат соединения + жизненный цикл ObdService.
class ConnectionController extends StateNotifier<ConnectionUiState> {
  ConnectionController() : super(const ConnectionUiState());

  ObdService? _service;
  ObdService? get service => _service;

  Future<void> _ensurePermissions(TransportKind kind) async {
    if (kind == TransportKind.wifi) return; // Wi-Fi не требует BT-разрешений
    if (Platform.isAndroid || Platform.isIOS) {
      await [
        Permission.bluetoothScan,
        Permission.bluetoothConnect,
        Permission.locationWhenInUse, // нужно для скана на старых Android
      ].request();
    }
  }

  Future<void> connect(TransportKind kind) async {
    try {
      state = state.copyWith(link: LinkState.connecting, error: null);
      await _ensurePermissions(kind);

      final transport = _buildTransport(kind);
      final service = ObdService(transport);
      _service = service;

      // транслируем состояние линка в UI
      transport.linkState.listen((s) {
        state = state.copyWith(link: s);
        if (s == LinkState.disconnected) _service?.stopPolling();
      });

      await transport.connect();

      state = state.copyWith(link: LinkState.initializing);
      await service.initialize();

      service.startPolling();
      state = state.copyWith(link: LinkState.connected);
    } catch (e) {
      state = state.copyWith(link: LinkState.error, error: e.toString());
    }
  }

  Future<void> disconnect() async {
    await _service?.transport.disconnect();
    await _service?.dispose();
    _service = null;
    state = const ConnectionUiState();
  }
}

class ConnectionUiState {
  final LinkState link;
  final String? error;
  const ConnectionUiState({this.link = LinkState.disconnected, this.error});

  ConnectionUiState copyWith({LinkState? link, String? error}) =>
      ConnectionUiState(link: link ?? this.link, error: error);
}

final connectionProvider =
    StateNotifierProvider<ConnectionController, ConnectionUiState>(
  (ref) => ConnectionController(),
);

/// Стрим телеметрии для дашборда.
final telemetryProvider = StreamProvider<Telemetry>((ref) {
  final ctrl = ref.watch(connectionProvider.notifier);
  final svc = ctrl.service;
  if (svc == null) return const Stream.empty();
  return svc.telemetry;
});


// ===== из gauge_widget.dart =====

/// Аналоговый круговой прибор (стрелка), как в Torque.
class GaugeWidget extends StatelessWidget {
  final String label;
  final String unit;
  final double value;
  final double maxValue;
  final Color color;

  const GaugeWidget({
    super.key,
    required this.label,
    required this.unit,
    required this.value,
    required this.maxValue,
    this.color = Colors.cyanAccent,
  });

  @override
  Widget build(BuildContext context) {
    return AspectRatio(
      aspectRatio: 1,
      child: CustomPaint(
        painter: _GaugePainter(value.clamp(0, maxValue), maxValue, color),
        child: Center(
          child: Column(
            mainAxisSize: MainAxisSize.min,
            children: [
              const SizedBox(height: 36),
              Text(value.toStringAsFixed(0),
                  style: TextStyle(
                      color: color,
                      fontSize: 30,
                      fontWeight: FontWeight.bold)),
              Text(unit,
                  style: const TextStyle(color: Colors.white54, fontSize: 12)),
              const SizedBox(height: 4),
              Text(label,
                  style: const TextStyle(color: Colors.white70, fontSize: 13)),
            ],
          ),
        ),
      ),
    );
  }
}

class _GaugePainter extends CustomPainter {
  final double value;
  final double maxValue;
  final Color color;
  _GaugePainter(this.value, this.maxValue, this.color);

  // дуга от 135° до 405° (270° полного хода)
  static const _start = 135 * pi / 180;
  static const _sweep = 270 * pi / 180;

  @override
  void paint(Canvas canvas, Size size) {
    final center = Offset(size.width / 2, size.height / 2);
    final radius = size.width / 2 - 10;
    final rect = Rect.fromCircle(center: center, radius: radius);

    // фон дуги
    final bg = Paint()
      ..color = Colors.white12
      ..style = PaintingStyle.stroke
      ..strokeWidth = 10
      ..strokeCap = StrokeCap.round;
    canvas.drawArc(rect, _start, _sweep, false, bg);

    // заполненная часть
    final fillAngle = _sweep * (value / maxValue);
    final fg = Paint()
      ..color = color
      ..style = PaintingStyle.stroke
      ..strokeWidth = 10
      ..strokeCap = StrokeCap.round;
    canvas.drawArc(rect, _start, fillAngle, false, fg);

    // стрелка
    final needleAngle = _start + fillAngle;
    final needleEnd = Offset(
      center.dx + (radius - 18) * cos(needleAngle),
      center.dy + (radius - 18) * sin(needleAngle),
    );
    final needle = Paint()
      ..color = Colors.redAccent
      ..strokeWidth = 3
      ..strokeCap = StrokeCap.round;
    canvas.drawLine(center, needleEnd, needle);
    canvas.drawCircle(center, 5, Paint()..color = Colors.redAccent);
  }

  @override
  bool shouldRepaint(_GaugePainter old) => old.value != value;
}


// ===== из dashboard_screen.dart =====

class DashboardScreen extends ConsumerWidget {
  const DashboardScreen({super.key});

  @override
  Widget build(BuildContext context, WidgetRef ref) {
    final conn = ref.watch(connectionProvider);
    final kind = ref.watch(transportKindProvider);

    return Scaffold(
      backgroundColor: const Color(0xFF0E1116),
      appBar: AppBar(
        backgroundColor: const Color(0xFF161B22),
        title: const Text("OBD Scanner"),
        actions: [_StatusChip(conn.link)],
      ),
      body: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          children: [
            _TransportSelector(kind: kind),
            const SizedBox(height: 12),
            _ConnectBar(state: conn),
            if (conn.error != null) ...[
              const SizedBox(height: 8),
              Text(conn.error!,
                  style: const TextStyle(color: Colors.redAccent)),
            ],
            const SizedBox(height: 16),
            const Expanded(child: _GaugesGrid()),
            const _DtcPanel(),
          ],
        ),
      ),
    );
  }
}

class _TransportSelector extends ConsumerWidget {
  final TransportKind kind;
  const _TransportSelector({required this.kind});

  @override
  Widget build(BuildContext context, WidgetRef ref) {
    return SegmentedButton<TransportKind>(
      segments: const [
        ButtonSegment(value: TransportKind.ble, label: Text("BLE")),
        ButtonSegment(value: TransportKind.btClassic, label: Text("BT")),
        ButtonSegment(value: TransportKind.wifi, label: Text("Wi-Fi")),
      ],
      selected: {kind},
      onSelectionChanged: (s) =>
          ref.read(transportKindProvider.notifier).state = s.first,
    );
  }
}

class _ConnectBar extends ConsumerWidget {
  final ConnectionUiState state;
  const _ConnectBar({required this.state});

  @override
  Widget build(BuildContext context, WidgetRef ref) {
    final isConnected = state.link == LinkState.connected;
    final busy = state.link == LinkState.connecting ||
        state.link == LinkState.initializing ||
        state.link == LinkState.scanning;

    return SizedBox(
      width: double.infinity,
      child: FilledButton.icon(
        icon: Icon(isConnected ? Icons.link_off : Icons.bluetooth_searching),
        label: Text(busy
            ? "Подключение…"
            : isConnected
                ? "Отключить"
                : "Подключить адаптер"),
        onPressed: busy
            ? null
            : () {
                final ctrl = ref.read(connectionProvider.notifier);
                if (isConnected) {
                  ctrl.disconnect();
                } else {
                  ctrl.connect(ref.read(transportKindProvider));
                }
              },
      ),
    );
  }
}

class _GaugesGrid extends ConsumerWidget {
  const _GaugesGrid();

  @override
  Widget build(BuildContext context, WidgetRef ref) {
    final tele = ref.watch(telemetryProvider);

    return tele.when(
      data: (t) => GridView.count(
        crossAxisCount: 2,
        mainAxisSpacing: 16,
        crossAxisSpacing: 16,
        children: [
          GaugeWidget(
              label: "Обороты",
              unit: "об/мин",
              value: (t.rpm ?? 0).toDouble(),
              maxValue: 8000,
              color: Colors.cyanAccent),
          GaugeWidget(
              label: "Скорость",
              unit: "км/ч",
              value: (t.speed ?? 0).toDouble(),
              maxValue: 240,
              color: Colors.greenAccent),
          GaugeWidget(
              label: "Темп. ОЖ",
              unit: "°C",
              value: (t.coolant ?? 0).toDouble(),
              maxValue: 130,
              color: Colors.orangeAccent),
          GaugeWidget(
              label: "Нагрузка",
              unit: "%",
              value: t.load ?? 0,
              maxValue: 100,
              color: Colors.purpleAccent),
          GaugeWidget(
              label: "Наддув (MAP)",
              unit: "кПа",
              value: (t.map ?? 0).toDouble(),
              maxValue: 250,
              color: Colors.tealAccent),
          GaugeWidget(
              label: "Топливо",
              unit: "%",
              value: t.fuelLevel ?? 0,
              maxValue: 100,
              color: Colors.amberAccent),
          GaugeWidget(
              label: "Расход",
              unit: "л/ч",
              value: t.fuelRate ?? 0,
              maxValue: 30,
              color: Colors.lightBlueAccent),
          GaugeWidget(
              label: "Темп. впуска",
              unit: "°C",
              value: (t.intakeTemp ?? 0).toDouble(),
              maxValue: 90,
              color: Colors.pinkAccent),
        ],
      ),
      loading: () => const Center(child: CircularProgressIndicator()),
      error: (e, _) => Center(
        child: Text("Нет данных: $e",
            style: const TextStyle(color: Colors.redAccent)),
      ),
    );
  }
}

class _DtcPanel extends ConsumerStatefulWidget {
  const _DtcPanel();
  @override
  ConsumerState<_DtcPanel> createState() => _DtcPanelState();
}

class _DtcPanelState extends ConsumerState<_DtcPanel> {
  List<String> _codes = [];
  bool _loading = false;

  Future<void> _read() async {
    final svc = ref.read(connectionProvider.notifier).service;
    if (svc == null) return;
    setState(() => _loading = true);
    try {
      final codes = await svc.readDtc();
      setState(() => _codes = codes);
    } finally {
      setState(() => _loading = false);
    }
  }

  Future<void> _clear() async {
    final svc = ref.read(connectionProvider.notifier).service;
    if (svc == null) return;

    // ОБЯЗАТЕЛЬНОЕ подтверждение перед записью в шину
    final ok = await showDialog<bool>(
      context: context,
      builder: (_) => AlertDialog(
        title: const Text("Сбросить ошибки?"),
        content: const Text(
            "Check Engine погаснет, freeze frame будет стёрт. "
            "Автомобиль должен стоять (V=0). Продолжить?"),
        actions: [
          TextButton(
              onPressed: () => Navigator.pop(context, false),
              child: const Text("Отмена")),
          FilledButton(
              onPressed: () => Navigator.pop(context, true),
              child: const Text("Сбросить")),
        ],
      ),
    );
    if (ok != true) return;

    try {
      await svc.clearDtc(userConfirmed: true);
      setState(() => _codes = []);
      if (mounted) {
        ScaffoldMessenger.of(context).showSnackBar(
            const SnackBar(content: Text("Ошибки сброшены")));
      }
    } catch (e) {
      if (mounted) {
        ScaffoldMessenger.of(context).showSnackBar(
            SnackBar(content: Text("$e"), backgroundColor: Colors.red));
      }
    }
  }

  @override
  Widget build(BuildContext context) {
    final connected =
        ref.watch(connectionProvider).link == LinkState.connected;

    return Card(
      color: const Color(0xFF161B22),
      child: Padding(
        padding: const EdgeInsets.all(12),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                const Text("Ошибки (DTC)",
                    style: TextStyle(color: Colors.white, fontSize: 16)),
                const Spacer(),
                TextButton(
                    onPressed: connected && !_loading ? _read : null,
                    child: const Text("Прочитать")),
                TextButton(
                    onPressed: connected && _codes.isNotEmpty ? _clear : null,
                    child: const Text("Сбросить",
                        style: TextStyle(color: Colors.redAccent))),
              ],
            ),
            if (_loading) const LinearProgressIndicator(),
            if (_codes.isEmpty && !_loading)
              const Text("Кодов нет",
                  style: TextStyle(color: Colors.white38))
            else
              Wrap(
                spacing: 8,
                children: _codes
                    .map((c) => Chip(
                          label: Text(c),
                          backgroundColor: Colors.red.shade900,
                          labelStyle: const TextStyle(color: Colors.white),
                        ))
                    .toList(),
              ),
          ],
        ),
      ),
    );
  }
}

class _StatusChip extends StatelessWidget {
  final LinkState link;
  const _StatusChip(this.link);

  @override
  Widget build(BuildContext context) {
    final map = {
      LinkState.disconnected: ("Отключено", Colors.grey),
      LinkState.scanning: ("Поиск…", Colors.amber),
      LinkState.connecting: ("Подключение…", Colors.amber),
      LinkState.initializing: ("Инициализация…", Colors.amber),
      LinkState.connected: ("Онлайн", Colors.green),
      LinkState.error: ("Ошибка", Colors.red),
    };
    final (text, color) = map[link]!;
    return Padding(
      padding: const EdgeInsets.only(right: 12),
      child: Chip(
        label: Text(text, style: const TextStyle(fontSize: 12)),
        backgroundColor: color.withOpacity(0.2),
        side: BorderSide(color: color),
      ),
    );
  }
}


// ===== из main.dart =====

void main() {
  runApp(const ProviderScope(child: ObdApp()));
}

class ObdApp extends StatelessWidget {
  const ObdApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: "OBD Scanner",
      debugShowCheckedModeBanner: false,
      theme: ThemeData(
        useMaterial3: true,
        brightness: Brightness.dark,
        colorSchemeSeed: Colors.cyan,
      ),
      home: const DashboardScreen(),
    );
  }
}
