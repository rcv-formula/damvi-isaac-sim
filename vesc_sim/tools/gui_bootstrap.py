"""Execute once via the existing GUI Console command field (script folders).

Enables the installed localhost-only Isaac Python Server; does not reopen stage.
"""
import omni.kit.app
omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate(
    'isaacsim.code_editor.python_server',True)
print('[VESC_GUI] localhost Python Server enabled in existing GUI')
