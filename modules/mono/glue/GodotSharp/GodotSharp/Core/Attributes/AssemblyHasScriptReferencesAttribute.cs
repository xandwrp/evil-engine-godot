using System;

#nullable enable

namespace Godot
{
    [AttributeUsage(AttributeTargets.Assembly)]
    public sealed class AssemblyHasScriptReferencesAttribute : Attribute
    {
        public string[] AssemblyNames { get; }

        public AssemblyHasScriptReferencesAttribute(string[] assemblyNames)
        {
            AssemblyNames = assemblyNames;
        }
    }
}

#nullable restore
